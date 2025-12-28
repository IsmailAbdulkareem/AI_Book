"""Skill 7: CLI Runner - Terminal commands, directory ops, ingestion scripts"""

import subprocess
import os
from pathlib import Path
from typing import Dict, Any, Optional, List


class CliRunner:
    """Executes terminal commands, manages directories, runs scripts."""

    def __init__(self, repo_root: str = "."):
        self.repo_root = Path(repo_root)
        self.cwd = self.repo_root

    def run_command(self, cmd: str, shell: bool = True) -> Dict[str, Any]:
        """Execute a shell command."""
        try:
            result = subprocess.run(
                cmd,
                shell=shell,
                cwd=str(self.cwd),
                capture_output=True,
                text=True,
                timeout=300
            )
            
            return {
                "status": "success" if result.returncode == 0 else "error",
                "returncode": result.returncode,
                "stdout": result.stdout,
                "stderr": result.stderr
            }
        except subprocess.TimeoutExpired:
            return {"status": "error", "message": "Command timeout"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def run_npm(self, script: str, args: List[str] = []) -> Dict[str, Any]:
        """Run npm script."""
        cmd = f"npm run {script}" + (" " + " ".join(args) if args else "")
        return self.run_command(cmd)

    def run_python(self, script: str, args: List[str] = []) -> Dict[str, Any]:
        """Run Python script."""
        cmd = f"python {script}" + (" " + " ".join(args) if args else "")
        return self.run_command(cmd)

    def run_ingestion(self) -> Dict[str, Any]:
        """Run the full RAG ingestion pipeline."""
        try:
            from rag_ingestor import RagIngestor
            ingestor = RagIngestor(str(self.repo_root))
            result = ingestor.ingest_pipeline()
            return result
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def start_fastapi(self, port: int = 8000) -> Dict[str, Any]:
        """Start FastAPI backend."""
        cmd = f"uvicorn backend.main:app --reload --port {port}"
        return self.run_command(cmd)

    def mkdir(self, path: str) -> Dict[str, Any]:
        """Create directory."""
        try:
            (self.repo_root / path).mkdir(parents=True, exist_ok=True)
            return {"status": "success", "path": path}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def list_files(self, pattern: str = "*") -> Dict[str, Any]:
        """List files in current directory."""
        try:
            files = list(self.repo_root.glob(pattern))
            return {
                "status": "success",
                "files": [str(f.relative_to(self.repo_root)) for f in files]
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def change_directory(self, path: str) -> Dict[str, Any]:
        """Change working directory."""
        try:
            new_cwd = self.repo_root / path
            if new_cwd.exists():
                self.cwd = new_cwd
                return {"status": "success", "cwd": str(self.cwd)}
            else:
                return {"status": "error", "message": f"Path not found: {path}"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def execute_batch(self, commands: List[str]) -> Dict[str, Any]:
        """Execute multiple commands in sequence."""
        results = []
        for cmd in commands:
            result = self.run_command(cmd)
            results.append(result)
            if result["status"] == "error":
                return {"status": "error", "failed_at": cmd, "results": results}
        
        return {"status": "success", "executed": len(commands), "results": results}
