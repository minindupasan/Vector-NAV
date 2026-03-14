"""
VECTOR RAG — ROS2 Node
======================
Pure retrieval node. Finds relevant knowledge-base chunks for a query
and publishes the formatted context. Does NOT call any LLM.

Topics
------
  Subscribes : /rag/query    (vector_interfaces/RagQuery)    — query + optional top_k
  Publishes  : /rag/context  (vector_interfaces/RagContext)  — retrieved context + sources

Usage
-----
  ros2 run vector_rag rag_node
  ros2 topic pub /rag/query vector_interfaces/msg/RagQuery "{text: 'Where is the charging station?', top_k: 0}"
  ros2 topic echo /rag/context
"""

import rclpy
from rclpy.node import Node

from vector_interfaces.msg import RagQuery, RagContext
from vector_rag.rag_engine import RAGEngine


class RagNode(Node):
    """ROS2 node that wraps RAGEngine for pure semantic retrieval."""

    def __init__(self):
        super().__init__('rag_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('top_k', 3)
        self.declare_parameter('force_rebuild', False)

        self._default_top_k = self.get_parameter('top_k').get_parameter_value().integer_value
        force_rebuild        = self.get_parameter('force_rebuild').get_parameter_value().bool_value

        # ── RAG engine ────────────────────────────────────────────────────────
        self.get_logger().info('Loading RAG engine…')
        self._rag = RAGEngine(top_k=self._default_top_k)
        self._rag.load(force_rebuild=force_rebuild)
        self.get_logger().info('RAG engine ready.')

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._context_pub = self.create_publisher(RagContext, '/rag/context', 10)
        self.create_subscription(RagQuery, '/rag/query', self._on_query, 10)

        self.get_logger().info(
            'RagNode started.\n'
            '  Subscribes : /rag/query   (vector_interfaces/RagQuery)\n'
            '  Publishes  : /rag/context (vector_interfaces/RagContext)'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_query(self, msg: RagQuery) -> None:
        query = msg.text.strip()
        if not query:
            return

        # Per-message top_k override (0 = use node default)
        top_k = msg.top_k if msg.top_k > 0 else self._default_top_k
        self._rag.top_k = top_k

        self.get_logger().info(f'Query: "{query}"  top_k={top_k}')

        try:
            results = self._rag.retrieve(query)
            context = self._rag.format_context(results)
            sources = list({r['source'] for r in results})
        except Exception as e:
            self.get_logger().error(f'Retrieval failed: {e}')
            context = ''
            sources = []

        out = RagContext()
        out.query   = query
        out.context = context
        out.sources = sources
        self._context_pub.publish(out)
        self.get_logger().info(f'Published context from sources: {sources}')


def main(args=None):
    rclpy.init(args=args)
    node = RagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
