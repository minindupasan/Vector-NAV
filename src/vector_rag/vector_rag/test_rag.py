"""
VECTOR RAG — Retrieval Test CLI
================================
Standalone test for the RAG retrieval engine (no ROS2, no LLM needed).
Run: python3 test_rag.py
"""

from vector_rag.rag_engine import RAGEngine


def main():
    rag = RAGEngine(top_k=3)
    rag.load()

    print("=" * 60)
    print("  VECTOR RAG — Retrieval Test")
    print("  Type a query to see retrieved context chunks.")
    print("  Type 'quit' to exit.")
    print("=" * 60)

    while True:
        try:
            query = input("\nQuery: ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            break

        if not query:
            continue
        if query.lower() in ("quit", "exit", "q"):
            break

        results = rag.retrieve(query)
        print(f"\n--- Top {len(results)} chunks ---")
        for i, r in enumerate(results, 1):
            print(f"\n[{i}] Score: {r['score']:.3f} | Source: {r['source']}")
            print(r['text'])

        print("\n--- Formatted context ---")
        print(rag.format_context(results))


if __name__ == "__main__":
    main()
