"""Skill 9: Debugger - Error diagnosis, import troubleshooting, config validation"""

import subprocess
import traceback
from typing import Dict, Any, Optional
from pathlib import Path


class Debugger:
    """Diagnoses and fixes Python errors, imports, configurations."""

    def __init__(self, repo_root: str = "."):
        self.repo_root = Path(repo_root)

    def check_imports(self, module_name: str) -> Dict[str, Any]:
        """Check if a module can be imported."""
        try:
            __import__(module_name)
            return {"status": "success", "message": f"{module_name} imported successfully"}
        except ImportError as e:
            return {
                "status": "error",
                "error": str(e),
                "suggestion": self._suggest_install(module_name)
            }
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def validate_python_syntax(self, file_path: str) -> Dict[str, Any]:
        """Check Python file syntax."""
        try:
            import py_compile
            py_compile.compile(file_path, doraise=True)
            return {"status": "success", "message": f"{file_path} syntax valid"}
        except py_compile.PyCompileError as e:
            return {"status": "error", "error": str(e), "file": file_path}

    def validate_json(self, file_path: str) -> Dict[str, Any]:
        """Validate JSON file."""
        try:
            import json
            with open(file_path) as f:
                json.load(f)
            return {"status": "success", "message": f"{file_path} is valid JSON"}
        except Exception as e:
            return {"status": "error", "error": str(e), "file": file_path}

    def validate_env_file(self, env_file: str = ".env") -> Dict[str, Any]:
        """Validate .env file format."""
        try:
            env_path = self.repo_root / env_file
            if not env_path.exists():
                return {"status": "warning", "message": f"{env_file} not found"}
            
            content = env_path.read_text()
            lines = content.strip().split('\n')
            
            issues = []
            for i, line in enumerate(lines, 1):
                if not line or line.startswith('#'):
                    continue
                if '=' not in line:
                    issues.append(f"Line {i}: Missing '='")
            
            if issues:
                return {"status": "warning", "issues": issues}
            
            return {"status": "success", "message": f"{env_file} is valid"}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def check_dependencies(self, requirements_file: str = "requirements.txt") -> Dict[str, Any]:
        """Check if all required packages are installed."""
        try:
            req_path = self.repo_root / requirements_file
            if not req_path.exists():
                return {"status": "warning", "message": f"{requirements_file} not found"}
            
            packages = req_path.read_text().strip().split('\n')
            missing = []
            
            for package in packages:
                if package and not package.startswith('#'):
                    pkg_name = package.split('==')[0].split('>=')[0].split('<=')[0].strip()
                    result = self.check_imports(pkg_name)
                    if result["status"] != "success":
                        missing.append(pkg_name)
            
            if missing:
                return {
                    "status": "warning",
                    "missing_packages": missing,
                    "install_cmd": f"pip install {' '.join(missing)}"
                }
            
            return {"status": "success", "message": "All dependencies installed"}
        except Exception as e:
            return {"status": "error", "error": str(e)}

    def diagnose_issue(self, error_message: str) -> Dict[str, Any]:
        """Diagnose an error and suggest fixes."""
        diagnostics = {
            "error": error_message,
            "suggestions": []
        }
        
        # Common error patterns
        if "ModuleNotFoundError" in error_message or "ImportError" in error_message:
            diagnostics["suggestions"].append("Run: pip install <module_name>")
        elif "PermissionError" in error_message:
            diagnostics["suggestions"].append("Check file permissions or run with sudo")
        elif "ConnectionError" in error_message:
            diagnostics["suggestions"].append("Check if Qdrant/API is running")
        elif "KeyError" in error_message:
            diagnostics["suggestions"].append("Check your .env variables")
        else:
            diagnostics["suggestions"].append("Check error traceback for details")
        
        return {"status": "warning", "diagnostics": diagnostics}

    def _suggest_install(self, module_name: str) -> str:
        """Suggest pip install command for a module."""
        mapping = {
            "fastapi": "pip install fastapi uvicorn[standard]",
            "cohere": "pip install cohere",
            "openai": "pip install openai",
            "qdrant_client": "pip install qdrant-client",
            "pydantic": "pip install pydantic",
        }
        return mapping.get(module_name, f"pip install {module_name}")

    def full_diagnostic(self) -> Dict[str, Any]:
        """Run full project diagnostic."""
        results = {
            "imports": self.check_imports("fastapi"),
            "requirements": self.check_dependencies(),
            "env": self.validate_env_file(),
        }
        
        overall_status = "success"
        if any(r.get("status") in ["error", "warning"] for r in results.values()):
            overall_status = "warning"
        
        return {"status": overall_status, "diagnostics": results}
