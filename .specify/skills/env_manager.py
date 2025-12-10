"""Skill 1: Environment Manager - Python env, packages, virtualenv, .env"""

import os
import subprocess
import json
from pathlib import Path
from typing import List, Dict, Any, Optional


class EnvManager:
    """Manages Python environment, dependencies, and .env configuration."""

    def __init__(self, repo_root: str = "."):
        self.repo_root = Path(repo_root)
        self.venv_path = self.repo_root / ".venv"
        self.env_file = self.repo_root / ".env"

    def create_venv(self) -> Dict[str, Any]:
        """Create a Python virtual environment."""
        try:
            subprocess.run([
                "python", "-m", "venv", str(self.venv_path)
            ], check=True)
            return {"status": "success", "message": f"Created venv at {self.venv_path}"}
        except subprocess.CalledProcessError as e:
            return {"status": "error", "message": str(e)}

    def install_packages(self, packages: List[str]) -> Dict[str, Any]:
        """Install pip packages in the venv."""
        pip_exe = self.venv_path / "Scripts" / "pip" if os.name == "nt" else self.venv_path / "bin" / "pip"
        try:
            subprocess.run([str(pip_exe), "install"] + packages, check=True)
            return {"status": "success", "packages_installed": packages}
        except subprocess.CalledProcessError as e:
            return {"status": "error", "message": str(e)}

    def generate_env(self, config: Dict[str, str]) -> Dict[str, Any]:
        """Generate .env file from config dict (secure secrets handling)."""
        env_content = "\n".join([f"{k}={v}" for k, v in config.items()])
        try:
            self.env_file.write_text(env_content)
            return {"status": "success", "env_file": str(self.env_file)}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def check_dependencies(self) -> Dict[str, Any]:
        """Check system dependencies (Python, Node, git)."""
        deps = {
            "python": self._check_command("python --version"),
            "pip": self._check_command("pip --version"),
            "node": self._check_command("node --version"),
            "npm": self._check_command("npm --version"),
            "git": self._check_command("git --version"),
        }
        return {"status": "success", "dependencies": deps}

    def _check_command(self, cmd: str) -> Optional[str]:
        """Check if command exists and return output."""
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
            return result.stdout.strip() if result.returncode == 0 else None
        except:
            return None

    def list_installed(self) -> Dict[str, Any]:
        """List all installed packages in current env."""
        try:
            result = subprocess.run(["pip", "list", "--format", "json"], capture_output=True, text=True)
            packages = json.loads(result.stdout)
            return {"status": "success", "packages": packages}
        except Exception as e:
            return {"status": "error", "message": str(e)}
