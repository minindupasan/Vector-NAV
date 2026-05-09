"""
VECTOR NAV — RAG Engine
=======================
Handles document loading, chunking, embedding, FAISS indexing, and retrieval.
This is the core logic used by both the standalone CLI and the ROS2 node.
"""

import os
import json
import time
import faiss
import numpy as np
from pathlib import Path
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from tokenizers import Tokenizer


# ─── Constants ────────────────────────────────────────────────────────────────

KNOWLEDGE_BASE_DIR = Path(__file__).parent / "knowledge_base"
INDEX_CACHE_PATH   = Path(__file__).parent / "data" / "faiss.index"
CHUNKS_CACHE_PATH  = Path(__file__).parent / "data" / "chunks.json"

# Try to find ONNX/TRT model in package data or source fallback
ONNX_MODEL_DIR = Path(__file__).parent / "data" / "onnx"
if not (ONNX_MODEL_DIR / "model.engine").exists():
    # Fallback for colcon symlink/install environments
    _src_path = Path("/home/admin/vector_nav/src/vector_rag/vector_rag/data/onnx")
    if _src_path.exists():
        ONNX_MODEL_DIR = _src_path

CHUNK_SIZE         = 200                   # words per chunk
CHUNK_OVERLAP      = 40                    # words overlap between chunks
TOP_K              = 3                     # how many chunks to retrieve


# ─── Text Chunking ────────────────────────────────────────────────────────────

def chunk_text(text: str, source: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[dict]:
    """
    Split text into overlapping word-based chunks.
    Each chunk is a dict with 'text' and 'source' fields.

    Example:
        words = [w1, w2, ..., w300]
        chunk 1: w1   → w200
        chunk 2: w160 → w360   (overlap of 40 words)
    """
    words  = text.split()
    chunks = []
    start  = 0

    while start < len(words):
        end        = min(start + chunk_size, len(words))
        chunk_text = " ".join(words[start:end])
        chunks.append({"text": chunk_text, "source": source})
        if end == len(words):
            break
        start += chunk_size - overlap  # slide window with overlap

    return chunks


def load_documents(kb_dir: Path = KNOWLEDGE_BASE_DIR) -> list[dict]:
    """Load all .txt files from the knowledge base directory into chunks."""
    all_chunks = []

    txt_files = list(kb_dir.glob("*.txt"))
    if not txt_files:
        raise FileNotFoundError(f"No .txt files found in {kb_dir}")

    for filepath in sorted(txt_files):
        text = filepath.read_text(encoding="utf-8").strip()
        if not text:
            continue
        chunks = chunk_text(text, source=filepath.name)
        all_chunks.extend(chunks)
        print(f"  Loaded '{filepath.name}' → {len(chunks)} chunks")

    return all_chunks


# ─── Embedding & Indexing ─────────────────────────────────────────────────────

def build_index(chunks: list[dict], engine: 'RAGEngine') -> faiss.IndexFlatIP:
    """
    Embed all chunks and build a FAISS inner-product index.
    Vectors are L2-normalised so inner product == cosine similarity.
    """
    texts      = [c["text"] for c in chunks]
    embeddings = engine._embed_batch(texts)
    embeddings = np.array(embeddings, dtype=np.float32)

    dim   = embeddings.shape[1]
    index = faiss.IndexFlatIP(dim)   # Inner Product (cosine after normalisation)
    index.add(embeddings)

    return index


def save_cache(index: faiss.IndexFlatIP, chunks: list[dict]) -> None:
    """Save FAISS index and chunks list to disk for fast reload."""
    INDEX_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
    faiss.write_index(index, str(INDEX_CACHE_PATH))
    with open(CHUNKS_CACHE_PATH, "w") as f:
        json.dump(chunks, f)
    print(f"  Cache saved → {INDEX_CACHE_PATH}")


def load_cache() -> tuple[faiss.IndexFlatIP, list[dict]]:
    """Load FAISS index and chunks from disk cache."""
    index  = faiss.read_index(str(INDEX_CACHE_PATH))
    with open(CHUNKS_CACHE_PATH) as f:
        chunks = json.load(f)
    return index, chunks


def cache_is_valid() -> bool:
    """Check if cache exists and is newer than all knowledge base files."""
    if not INDEX_CACHE_PATH.exists() or not CHUNKS_CACHE_PATH.exists():
        return False
    cache_mtime = INDEX_CACHE_PATH.stat().st_mtime
    for txt_file in KNOWLEDGE_BASE_DIR.glob("*.txt"):
        if txt_file.stat().st_mtime > cache_mtime:
            return False  # a document was updated, rebuild
    return True


# ─── RAG Engine Class ─────────────────────────────────────────────────────────

class BGE_TRTRunner:
    def __init__(self, engine_path: str, max_batch=1, max_seq_len=256):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        
        self.inputs = []
        self.outputs = []
        
        self.context.set_input_shape("input_ids", (max_batch, max_seq_len))
        self.context.set_input_shape("attention_mask", (max_batch, max_seq_len))
        
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self.context.get_tensor_shape(name)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            size = trt.volume(shape) * np.dtype(dtype).itemsize
            
            host_mem = cuda.pagelocked_empty(trt.volume(shape), dtype)
            device_mem = cuda.mem_alloc(size)
            self.context.set_tensor_address(name, int(device_mem))
            
            tensor_info = {
                "name": name,
                "host": host_mem,
                "device": device_mem,
                "shape": shape,
                "dtype": dtype
            }
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append(tensor_info)
            else:
                self.outputs.append(tensor_info)

    def run(self, input_ids_np, attention_mask_np):
        shape = input_ids_np.shape
        self.context.set_input_shape("input_ids", shape)
        self.context.set_input_shape("attention_mask", shape)
        
        for inp in self.inputs:
            if inp["name"] == "input_ids":
                np.copyto(inp["host"][:input_ids_np.size], input_ids_np.ravel().astype(inp["dtype"]))
            elif inp["name"] == "attention_mask":
                np.copyto(inp["host"][:attention_mask_np.size], attention_mask_np.ravel().astype(inp["dtype"]))
            cuda.memcpy_htod_async(inp["device"], inp["host"], self.stream)
            
        self.context.execute_async_v3(self.stream.handle)
        
        for out in self.outputs:
            out_shape = self.context.get_tensor_shape(out["name"])
            out["current_shape"] = out_shape
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
            
        self.stream.synchronize()
        
        out_dict = {}
        for out in self.outputs:
            out_dict[out["name"]] = out["host"][:trt.volume(out["current_shape"])].reshape(out["current_shape"]).copy()
            
        return out_dict


class RAGEngine:
    """
    Main RAG engine using TensorRT.
    """

    def __init__(self, top_k: int = TOP_K):
        self.top_k     = top_k
        self.runner    = None
        self.tokenizer = None
        self.index     = None
        self.chunks    = None
        self.ready     = False

    def load(self, force_rebuild: bool = False) -> None:
        """
        Load the TensorRT engine and FAISS index.
        Uses cached index if available and up-to-date, otherwise rebuilds.
        """
        print("\n[RAG] Loading TRT embedding engine...")
        t0 = time.time()
        
        engine_path = ONNX_MODEL_DIR / "model.engine"
        if not engine_path.exists():
            raise FileNotFoundError(f"TRT engine not found at {engine_path}. Run export script and trtexec first.")
            
        self.tokenizer = Tokenizer.from_file(str(ONNX_MODEL_DIR / "tokenizer.json"))
        self.tokenizer.enable_truncation(max_length=256)
        self.tokenizer.enable_padding(length=256)
        
        self.runner = BGE_TRTRunner(str(engine_path), max_batch=1, max_seq_len=256)
        print(f"[RAG] TRT engine loaded in {time.time() - t0:.1f}s")

        if not force_rebuild and cache_is_valid():
            print("[RAG] Loading index from cache...")
            self.index, self.chunks = load_cache()
            print(f"[RAG] Index loaded — {len(self.chunks)} chunks, {self.index.ntotal} vectors")
        else:
            print("[RAG] Building index from knowledge base...")
            t0           = time.time()
            self.chunks  = load_documents()
            self.index   = build_index(self.chunks, self)
            save_cache(self.index, self.chunks)
            print(f"[RAG] Index built in {time.time() - t0:.1f}s — {len(self.chunks)} chunks")

        self.ready = True
        print("[RAG] Ready.\n")

    def _embed_batch(self, texts: list[str]) -> np.ndarray:
        """Internal helper to generate embeddings using TensorRT.
        Processes texts one by one since we compiled for max_batch=1."""
        embeddings = []
        for text in texts:
            encoded = self.tokenizer.encode(text)
            input_ids = np.array(encoded.ids, dtype=np.int32).reshape(1, -1)
            attention_mask = np.array(encoded.attention_mask, dtype=np.int32).reshape(1, -1)
            
            outputs = self.runner.run(input_ids, attention_mask)
            
            # BGE uses CLS pooling (index 0) of last_hidden_state
            emb = outputs["last_hidden_state"][:, 0, :]
            # L2 Normalize
            norm = np.linalg.norm(emb, axis=1, keepdims=True)
            embeddings.append((emb / norm).squeeze())
            
        return np.array(embeddings, dtype=np.float32)

    def retrieve(self, query: str) -> list[dict]:
        """
        Find the top-K most relevant chunks for a query.
        Returns list of dicts with 'text', 'source', 'score'.
        """
        if not self.ready:
            raise RuntimeError("RAGEngine not loaded. Call .load() first.")

        query_vec = self._embed_batch([query])
        query_vec = np.array(query_vec, dtype=np.float32)

        scores, indices = self.index.search(query_vec, self.top_k)

        results = []
        for score, idx in zip(scores[0], indices[0]):
            if idx == -1:
                continue
            results.append({
                "text":   self.chunks[idx]["text"],
                "source": self.chunks[idx]["source"],
                "score":  float(score),
            })

        return results

    def format_context(self, results: list[dict]) -> str:
        """Format retrieved chunks into a single context string for the LLM prompt."""
        if not results:
            return "No relevant information found in the knowledge base."

        parts = []
        for i, r in enumerate(results, 1):
            parts.append(f"[Source: {r['source']}]\n{r['text']}")

        return "\n\n".join(parts)

