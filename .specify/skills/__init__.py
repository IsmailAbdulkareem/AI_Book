"""
AI-native Skills Architecture for Spec-Driven Development.

Skills are composable, autonomous agents that handle specific tasks:
- env_manager: Python environment & dependencies
- docusaurus_builder: Docs & site generation
- rag_ingestor: Document chunking & embedding
- vector_db_handler: Qdrant vector database operations
- chat_engine: LLM + retrieval-augmented generation
- fastapi_builder: Backend API scaffold & management
- cli_runner: Terminal command execution
- chat_ui_builder: React chat widget for Docusaurus
- debugger: Error diagnosis & fixes
- repo_manager: Git operations

Usage:
  from skills import SkillRegistry
  registry = SkillRegistry()
  registry.invoke('env_manager', action='install', packages=['fastapi', 'cohere'])
"""

__version__ = "0.1.0"
__all__ = [
    "EnvManager",
    "DocusaurusBuilder",
    "RagIngestor",
    "VectorDbHandler",
    "ChatEngine",
    "FastApiBuilder",
    "CliRunner",
    "ChatUiBuilder",
    "Debugger",
    "RepoManager",
    "SkillRegistry",
]
