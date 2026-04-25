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
from sentence_transformers import SentenceTransformer


# ─── Constants ────────────────────────────────────────────────────────────────

KNOWLEDGE_BASE_DIR = Path(__file__).parent / "knowledge_base"
INDEX_CACHE_PATH   = Path(__file__).parent / "data" / "faiss.index"
CHUNKS_CACHE_PATH  = Path(__file__).parent / "data" / "chunks.json"

EMBEDDING_MODEL    = "all-MiniLM-L6-v2"   # ~90MB, fast, good quality
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

def build_index(chunks: list[dict], model: SentenceTransformer) -> faiss.IndexFlatIP:
    """
    Embed all chunks and build a FAISS inner-product index.
    Vectors are L2-normalised so inner product == cosine similarity.
    """
    texts      = [c["text"] for c in chunks]
    embeddings = model.encode(texts, show_progress_bar=True, batch_size=32, normalize_embeddings=True)
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

class RAGEngine:
    """
    Main RAG engine.

    Usage:
        rag = RAGEngine()
        rag.load()
        context = rag.retrieve("Where is the loading bay?")
        answer  = rag.query("Where is the loading bay?")
    """

    def __init__(self, top_k: int = TOP_K):
        self.top_k    = top_k
        self.model    = None
        self.index    = None
        self.chunks   = None
        self.ready    = False

    def load(self, force_rebuild: bool = False) -> None:
        """
        Load the embedding model and FAISS index.
        Uses cached index if available and up-to-date, otherwise rebuilds.
        """
        print("\n[RAG] Loading embedding model...")
        t0          = time.time()
        self.model  = SentenceTransformer(EMBEDDING_MODEL, device='cpu')
        print(f"[RAG] Model loaded in {time.time() - t0:.1f}s")

        if not force_rebuild and cache_is_valid():
            print("[RAG] Loading index from cache...")
            self.index, self.chunks = load_cache()
            print(f"[RAG] Index loaded — {len(self.chunks)} chunks, {self.index.ntotal} vectors")
        else:
            print("[RAG] Building index from knowledge base...")
            t0           = time.time()
            self.chunks  = load_documents()
            self.index   = build_index(self.chunks, self.model)
            save_cache(self.index, self.chunks)
            print(f"[RAG] Index built in {time.time() - t0:.1f}s — {len(self.chunks)} chunks")

        self.ready = True
        print("[RAG] Ready.\n")

    def retrieve(self, query: str) -> list[dict]:
        """
        Find the top-K most relevant chunks for a query.
        Returns list of dicts with 'text', 'source', 'score'.
        """
        if not self.ready:
            raise RuntimeError("RAGEngine not loaded. Call .load() first.")

        query_vec = self.model.encode([query], normalize_embeddings=True)
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

