"""Skill Registry & Orchestrator - Central entry point for all skills"""

from typing import Dict, Any, Optional
from .env_manager import EnvManager
from .docusaurus_builder import DocusaurusBuilder
from .rag_ingestor import RagIngestor
from .vector_db_handler import VectorDbHandler
from .chat_engine import ChatEngine
from .fastapi_builder import FastApiBuilder
from .cli_runner import CliRunner
from .chat_ui_builder import ChatUiBuilder
from .debugger import Debugger
from .repo_manager import RepoManager


class SkillRegistry:
    """Central registry for all AI-native skills. Orchestrates skill invocation."""

    def __init__(self, repo_root: str = "."):
        self.repo_root = repo_root
        
        # Initialize all skills
        self.env_manager = EnvManager(repo_root)
        self.docusaurus_builder = DocusaurusBuilder(repo_root)
        self.rag_ingestor = RagIngestor(repo_root)
        self.vector_db_handler = VectorDbHandler()
        self.chat_engine = ChatEngine(self.vector_db_handler)
        self.fastapi_builder = FastApiBuilder(repo_root)
        self.cli_runner = CliRunner(repo_root)
        self.chat_ui_builder = ChatUiBuilder(repo_root)
        self.debugger = Debugger(repo_root)
        self.repo_manager = RepoManager(repo_root)

    def invoke(self, skill_name: str, action: str, **kwargs) -> Dict[str, Any]:
        """
        Invoke a skill with an action.
        
        Example:
            registry.invoke('env_manager', action='install_packages', packages=['fastapi', 'cohere'])
        """
        skill = self._get_skill(skill_name)
        if not skill:
            return {"status": "error", "message": f"Skill '{skill_name}' not found"}
        
        method = getattr(skill, action, None)
        if not method:
            return {"status": "error", "message": f"Action '{action}' not found in {skill_name}"}
        
        try:
            return method(**kwargs)
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def list_skills(self) -> Dict[str, list]:
        """List all available skills and their actions."""
        skills = {
            "env_manager": ["create_venv", "install_packages", "generate_env", "check_dependencies", "list_installed"],
            "docusaurus_builder": ["create_doc_page", "edit_sidebar", "run_dev_server", "build_site", "validate_site", "deploy_site"],
            "rag_ingestor": ["read_documents", "chunk_documents", "generate_embeddings", "upsert_to_qdrant", "ingest_pipeline"],
            "vector_db_handler": ["connect", "create_collection", "similarity_search", "delete_collection", "list_collections", "get_collection_info", "health_check", "troubleshoot"],
            "chat_engine": ["initialize", "embed_query", "retrieve_context", "generate_response", "chat", "stream_response"],
            "fastapi_builder": ["scaffold_backend", "add_endpoint", "run_server", "add_cors", "validate_syntax", "generate_requirements"],
            "cli_runner": ["run_command", "run_npm", "run_python", "run_ingestion", "start_fastapi", "mkdir", "list_files", "change_directory", "execute_batch"],
            "chat_ui_builder": ["create_chat_widget", "create_chat_styles", "embed_in_docusaurus", "add_tailwind"],
            "debugger": ["check_imports", "validate_python_syntax", "validate_json", "validate_env_file", "check_dependencies", "diagnose_issue", "full_diagnostic"],
            "repo_manager": ["get_status", "create_branch", "switch_branch", "commit", "push", "pull", "get_branches", "get_log", "check_merge_conflicts", "resolve_conflict"],
        }
        return skills

    def _get_skill(self, skill_name: str):
        """Get skill instance by name."""
        skills_map = {
            "env_manager": self.env_manager,
            "docusaurus_builder": self.docusaurus_builder,
            "rag_ingestor": self.rag_ingestor,
            "vector_db_handler": self.vector_db_handler,
            "chat_engine": self.chat_engine,
            "fastapi_builder": self.fastapi_builder,
            "cli_runner": self.cli_runner,
            "chat_ui_builder": self.chat_ui_builder,
            "debugger": self.debugger,
            "repo_manager": self.repo_manager,
        }
        return skills_map.get(skill_name)

    def run_setup_workflow(self) -> Dict[str, Any]:
        """Complete setup workflow: env → doc build → backend scaffold → widget creation."""
        workflow_steps = []
        
        # Step 1: Environment setup
        workflow_steps.append(("env_manager", self.env_manager.check_dependencies()))
        
        # Step 2: Build Docusaurus site
        workflow_steps.append(("docusaurus_builder", self.docusaurus_builder.build_site()))
        
        # Step 3: Backend scaffold
        workflow_steps.append(("fastapi_builder", self.fastapi_builder.scaffold_backend()))
        
        # Step 4: Chat UI
        workflow_steps.append(("chat_ui_builder", self.chat_ui_builder.create_chat_widget()))
        
        return {
            "status": "success",
            "workflow": "complete",
            "steps": workflow_steps
        }

    def run_ingestion_workflow(self, collection_name: str = "docs") -> Dict[str, Any]:
        """Complete ingestion workflow: read docs → chunk → embed → upsert to Qdrant."""
        try:
            # Step 1: Connect to Qdrant
            self.vector_db_handler.connect()
            
            # Step 2: Run ingestion pipeline
            result = self.rag_ingestor.ingest_pipeline(collection_name)
            
            return {
                "status": result.get("status"),
                "message": "Ingestion complete",
                "details": result
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def run_chat_workflow(self, query: str, collection_name: str = "docs") -> Dict[str, Any]:
        """Complete chat workflow: embed query → retrieve → generate response."""
        try:
            return self.chat_engine.chat(query, collection_name)
        except Exception as e:
            return {"status": "error", "message": str(e)}
