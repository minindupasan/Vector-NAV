#!/usr/bin/env python3
"""
Fix tied embeddings for Llama-3.2 models.

MLC in nano_llm:r36.4.0 expects lm_head.weight as a separate tensor,
but Llama-3.2 ties it with model.embed_tokens.weight (no lm_head.weight
in safetensors). This script copies embed_tokens → lm_head to untie them.
"""

import glob
import json
import os
import sys

try:
    from safetensors.torch import load_file, save_file
except ImportError:
    print("ERROR: safetensors not installed")
    sys.exit(1)


def find_model_snapshot(base_path, model_name):
    """Find the latest snapshot directory for a HF model."""
    pattern = os.path.join(base_path, f"models--{model_name.replace('/', '--')}", "snapshots", "*")
    snapshots = sorted(glob.glob(pattern))
    return snapshots[-1] if snapshots else None


def fix_tied_embeddings(model_dir):
    """Add lm_head.weight if it's missing (tied to embed_tokens)."""
    # Check config for tie_word_embeddings
    config_path = os.path.join(model_dir, "config.json")
    if not os.path.exists(config_path):
        print(f"No config.json found in {model_dir}")
        return False

    with open(config_path) as f:
        config = json.load(f)

    if not config.get("tie_word_embeddings", False):
        print("Model does not use tied embeddings, no fix needed.")
        return True

    # Find safetensors files
    st_files = sorted(glob.glob(os.path.join(model_dir, "*.safetensors")))
    if not st_files:
        print("No safetensors files found yet (still downloading), skipping fix.")
        return True

    # Check all shards in the index are present before attempting fix
    index_path = os.path.join(model_dir, "model.safetensors.index.json")
    if os.path.exists(index_path):
        with open(index_path) as f:
            index = json.load(f)
        expected = set(index["weight_map"].values())
        present = {os.path.basename(f) for f in st_files}
        missing = expected - present
        if missing:
            print(f"Shards still downloading: {missing}, skipping fix.")
            return True

    # Check if lm_head.weight already exists in any shard
    embed_weight = None
    lm_head_exists = False

    for st_file in st_files:
        tensors = load_file(st_file)
        if "lm_head.weight" in tensors:
            lm_head_exists = True
            break
        if "model.embed_tokens.weight" in tensors:
            embed_weight = tensors["model.embed_tokens.weight"]
            embed_file = st_file

    if lm_head_exists:
        print("lm_head.weight already exists, no fix needed.")
        return True

    if embed_weight is None:
        print("ERROR: model.embed_tokens.weight not found in any shard.")
        return False

    # Add lm_head.weight to the shard containing embed_tokens
    print(f"Copying model.embed_tokens.weight -> lm_head.weight in {os.path.basename(embed_file)}")
    tensors = load_file(embed_file)
    tensors["lm_head.weight"] = tensors["model.embed_tokens.weight"].clone()
    save_file(tensors, embed_file)

    # Update the index file if it exists
    index_path = os.path.join(model_dir, "model.safetensors.index.json")
    if os.path.exists(index_path):
        with open(index_path) as f:
            index = json.load(f)
        shard_name = os.path.basename(embed_file)
        index["weight_map"]["lm_head.weight"] = shard_name
        with open(index_path, "w") as f:
            json.dump(index, f, indent=2)
        print(f"Updated index: lm_head.weight -> {shard_name}")

    print("Fix applied successfully.")
    return True


if __name__ == "__main__":
    model_name = sys.argv[1] if len(sys.argv) > 1 else "meta-llama/Llama-3.2-3B-Instruct"
    hf_cache = "/data/models/huggingface"

    # Fix HF cache snapshot (if already downloaded)
    model_dir = find_model_snapshot(hf_cache, model_name)
    if model_dir:
        print(f"Model dir (HF cache): {model_dir}")
        if not fix_tied_embeddings(model_dir):
            sys.exit(1)
    else:
        print(f"Model not yet cached for {model_name}, skipping HF cache fix.")

    # Fix mlc dist model path (where MLC build actually reads from)
    model_short = model_name.split("/")[-1]
    mlc_model_dir = f"/data/models/mlc/dist/models/{model_short}"
    if os.path.isdir(mlc_model_dir):
        print(f"Model dir (MLC dist): {mlc_model_dir}")
        if not fix_tied_embeddings(mlc_model_dir):
            sys.exit(1)
    else:
        print(f"MLC dist model path not found ({mlc_model_dir}), skipping.")
