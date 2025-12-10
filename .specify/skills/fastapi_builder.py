"""Skill 6: FastAPI Builder - Backend scaffold, endpoints, schemas, CORS"""

import json
from pathlib import Path
from typing import Dict, Any, List, Optional


class FastApiBuilder:
    """Scaffolds and manages FastAPI backend: create endpoints, schemas, CORS, run server."""

    def __init__(self, repo_root: str = ".", backend_dir: str = "backend"):
        self.repo_root = Path(repo_root)
        self.backend_dir = self.repo_root / backend_dir
        self.main_file = self.backend_dir / "main.py"
        self.requirements_file = self.backend_dir / "requirements.txt"

    def scaffold_backend(self) -> Dict[str, Any]:
        """Create basic FastAPI backend structure."""
        try:
            self.backend_dir.mkdir(exist_ok=True)
            (self.backend_dir / "__init__.py").touch()
            
            # Create main.py
            main_content = '''"""FastAPI Backend for AI-native Chatbot"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os

app = FastAPI(title="AI Chatbot Backend", version="0.1.0")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Update for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Schemas
class QueryRequest(BaseModel):
    query: str
    collection: Optional[str] = "docs"

class ChatResponse(BaseModel):
    response: str
    sources: List[str]
    model: str

# Endpoints
@app.get("/health")
def health():
    return {"status": "ok"}

@app.post("/chat", response_model=ChatResponse)
def chat(request: QueryRequest):
    """Chat with RAG-enabled chatbot."""
    # TODO: Integrate chat_engine
    return ChatResponse(
        response="Hello! Ask me about Physical AI.",
        sources=[],
        model="gpt-4"
    )

@app.post("/ingest")
def ingest():
    """Ingest documents into vector DB."""
    # TODO: Integrate rag_ingestor
    return {"status": "ingestion_started"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
'''
            self.main_file.write_text(main_content)
            
            return {"status": "success", "backend_dir": str(self.backend_dir)}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def add_endpoint(self, path: str, method: str, endpoint_name: str, request_model: Optional[str] = None, response_model: Optional[str] = None) -> Dict[str, Any]:
        """Add a new endpoint to main.py (simplified)."""
        try:
            content = self.main_file.read_text()
            
            endpoint_code = f"""
@app.{method.lower()}("{path}", name="{endpoint_name}")
def {endpoint_name}():
    # TODO: Implement
    return {{"status": "ok"}}
"""
            # Insert before main block
            if "__main__" in content:
                content = content.replace("if __name__", endpoint_code + "\nif __name__", 1)
            else:
                content += endpoint_code
            
            self.main_file.write_text(content)
            return {"status": "success", "endpoint": path}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def run_server(self, host: str = "0.0.0.0", port: int = 8000) -> Dict[str, Any]:
        """Start FastAPI server with uvicorn."""
        try:
            import subprocess
            subprocess.Popen(
                ["uvicorn", "backend.main:app", "--reload", f"--host", host, f"--port", str(port)],
                cwd=str(self.repo_root)
            )
            return {"status": "success", "message": f"Server running on {host}:{port}"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def add_cors(self, origins: List[str] = ["*"]) -> Dict[str, Any]:
        """Update CORS configuration."""
        try:
            content = self.main_file.read_text()
            cors_config = f'allow_origins={json.dumps(origins)}'
            if "allow_origins=" in content:
                content = content.replace('allow_origins=["*"]', cors_config)
            self.main_file.write_text(content)
            return {"status": "success", "origins": origins}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def validate_syntax(self) -> Dict[str, Any]:
        """Check Python syntax errors."""
        try:
            import py_compile
            py_compile.compile(str(self.main_file), doraise=True)
            return {"status": "success", "message": "No syntax errors"}
        except py_compile.PyCompileError as e:
            return {"status": "error", "message": str(e)}

    def generate_requirements(self, packages: List[str]) -> Dict[str, Any]:
        """Generate requirements.txt."""
        try:
            content = "\n".join(packages)
            self.requirements_file.write_text(content)
            return {"status": "success", "packages": len(packages)}
        except Exception as e:
            return {"status": "error", "message": str(e)}
