#!/usr/bin/env python3
"""
Smoke-test the LLM + RAG pipeline without ROS2.
Run: python3 scripts/test_pipeline.py
"""

import sys
import json
from pathlib import Path

# Make source packages importable without installing
sys.path.insert(0, str(Path(__file__).parent.parent / 'src' / 'vector_rag'))
sys.path.insert(0, str(Path(__file__).parent.parent / 'src' / 'vector_llm'))

from vector_rag.rag_engine import RAGEngine
from vector_llm.llm_engine import LLMEngine

TOOLS_FILE = Path(__file__).parent.parent / 'config' / 'tools.json'

QUERIES = [
    "Where is the charging station?",
    "Go to the loading bay",
    "What is the maximum payload of the robot?",
    "Stop the robot",
    "What should I do in an emergency?",
]


def main():
    print("=" * 60)
    print("  VECTOR NAV — Pipeline Smoke Test")
    print("=" * 60)

    # Load RAG
    print("\n[1/2] Loading RAG engine...")
    rag = RAGEngine(top_k=3)
    rag.load()

    # Load tools
    tools = []
    if TOOLS_FILE.exists():
        tools = json.loads(TOOLS_FILE.read_text())
        print(f"[2/2] Loaded {len(tools)} tools from {TOOLS_FILE.name}")
    else:
        print("[2/2] tools.json not found — running without function calling")

    llm = LLMEngine(tools=tools)

    print("\n" + "=" * 60)

    for query in QUERIES:
        print(f"\nInput : {query}")

        results = rag.retrieve(query)
        context = rag.format_context(results)
        sources = [r['source'] for r in results]
        print(f"RAG   : {len(results)} chunks from {sources}")

        result = llm.chat(query, context=context)

        if result['type'] == 'tool_call':
            print(f"Route : /llm/tool_call")
            print(f"Tool  : {result['name']}  args={result['arguments']}")
        else:
            print(f"Route : /tts/input")
            print(f"Answer: {result['content']}")

        print("-" * 60)


if __name__ == '__main__':
    main()
