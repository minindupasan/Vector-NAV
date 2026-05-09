#!/usr/bin/env python3
"""
Export BAAI/bge-small-en-v1.5 to ONNX for use with ONNXRuntime.
This allows RAG embedding generation without loading the full PyTorch/SentenceTransformers stack.
"""

import os
import torch
import torch.nn as nn
from pathlib import Path
from transformers import AutoModel, AutoTokenizer

MODEL_ID = "BAAI/bge-small-en-v1.5"
OUTPUT_DIR = Path(__file__).parent.parent / "src" / "vector_rag" / "vector_rag" / "data" / "onnx"

class BGEModelWrapper(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model
    def forward(self, input_ids, attention_mask):
        outputs = self.model(input_ids=input_ids, attention_mask=attention_mask)
        return outputs.last_hidden_state

def export():
    print(f"Loading {MODEL_ID} and tokenizer...")
    tokenizer = AutoTokenizer.from_pretrained(MODEL_ID)
    base_model = AutoModel.from_pretrained(MODEL_ID, attn_implementation="eager")
    model = BGEModelWrapper(base_model)
    model.eval()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    onnx_path = OUTPUT_DIR / "model.onnx"

    print(f"Exporting to {onnx_path}...")
    
    # Dummy input for tracing
    text = "This is a test sentence for ONNX export."
    inputs = tokenizer(text, return_tensors="pt")

    torch.onnx.export(
        model,
        (inputs["input_ids"], inputs["attention_mask"]),
        str(onnx_path),
        input_names=["input_ids", "attention_mask"],
        output_names=["last_hidden_state"],
        dynamic_axes={
            "input_ids": {0: "batch_size", 1: "sequence_length"},
            "attention_mask": {0: "batch_size", 1: "sequence_length"},
            "last_hidden_state": {0: "batch_size", 1: "sequence_length"},
        },
        opset_version=14,
        do_constant_folding=True,
    )
    
    # Save tokenizer as well
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    print("Export complete.")

if __name__ == "__main__":
    export()
