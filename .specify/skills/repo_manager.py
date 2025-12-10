"""Skill 10: Repo Manager - Git operations, branches, commits, push, merge conflicts"""

import subprocess
from pathlib import Path
from typing import Dict, Any, Optional, List


class RepoManager:
    """Manages git operations: branches, commits, push, conflict resolution."""

    def __init__(self, repo_root: str = "."):
        self.repo_root = Path(repo_root)

    def get_status(self) -> Dict[str, Any]:
        """Get git status."""
        try:
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                return {"status": "error", "message": "Not a git repository"}
            
            changed_files = result.stdout.strip().split('\n') if result.stdout else []
            return {"status": "success", "changed_files": len(changed_files), "files": changed_files}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def create_branch(self, branch_name: str) -> Dict[str, Any]:
        """Create and checkout a new branch."""
        try:
            subprocess.run(
                ["git", "checkout", "-b", branch_name],
                cwd=str(self.repo_root),
                check=True
            )
            return {"status": "success", "branch": branch_name}
        except subprocess.CalledProcessError as e:
            return {"status": "error", "message": str(e)}

    def switch_branch(self, branch_name: str) -> Dict[str, Any]:
        """Switch to an existing branch."""
        try:
            subprocess.run(
                ["git", "checkout", branch_name],
                cwd=str(self.repo_root),
                check=True
            )
            return {"status": "success", "branch": branch_name}
        except subprocess.CalledProcessError as e:
            return {"status": "error", "message": str(e)}

    def commit(self, message: str, files: Optional[List[str]] = None) -> Dict[str, Any]:
        """Commit changes."""
        try:
            if files:
                for file in files:
                    subprocess.run(
                        ["git", "add", file],
                        cwd=str(self.repo_root),
                        check=True
                    )
            else:
                subprocess.run(
                    ["git", "add", "."],
                    cwd=str(self.repo_root),
                    check=True
                )
            
            result = subprocess.run(
                ["git", "commit", "-m", message],
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                return {"status": "success", "message": message}
            else:
                return {"status": "error", "stderr": result.stderr}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def push(self, branch: Optional[str] = None) -> Dict[str, Any]:
        """Push commits to remote."""
        try:
            cmd = ["git", "push"]
            if branch:
                cmd.extend(["origin", branch])
            
            result = subprocess.run(
                cmd,
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                return {"status": "success", "message": "Pushed successfully"}
            else:
                return {"status": "error", "stderr": result.stderr}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def pull(self) -> Dict[str, Any]:
        """Pull latest changes from remote."""
        try:
            result = subprocess.run(
                ["git", "pull"],
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                return {"status": "success", "message": "Pulled successfully"}
            else:
                return {"status": "error", "stderr": result.stderr}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def get_branches(self) -> Dict[str, Any]:
        """List all branches."""
        try:
            result = subprocess.run(
                ["git", "branch", "-a"],
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            branches = [b.strip() for b in result.stdout.split('\n') if b.strip()]
            return {"status": "success", "branches": branches}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def get_log(self, limit: int = 5) -> Dict[str, Any]:
        """Get recent commit log."""
        try:
            result = subprocess.run(
                ["git", "log", f"--max-count={limit}", "--oneline"],
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            commits = result.stdout.strip().split('\n') if result.stdout else []
            return {"status": "success", "commits": commits}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def check_merge_conflicts(self) -> Dict[str, Any]:
        """Check for merge conflicts."""
        try:
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=str(self.repo_root),
                capture_output=True,
                text=True
            )
            
            conflicts = [line for line in result.stdout.split('\n') if line.startswith('UU') or line.startswith('AA') or line.startswith('DD')]
            
            if conflicts:
                return {"status": "warning", "conflicts": conflicts}
            else:
                return {"status": "success", "message": "No merge conflicts"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def resolve_conflict(self, file_path: str, strategy: str = "ours") -> Dict[str, Any]:
        """Resolve merge conflict (simple resolution)."""
        try:
            # Strategies: "ours", "theirs"
            if strategy == "ours":
                subprocess.run(["git", "checkout", "--ours", file_path], cwd=str(self.repo_root), check=True)
            elif strategy == "theirs":
                subprocess.run(["git", "checkout", "--theirs", file_path], cwd=str(self.repo_root), check=True)
            
            subprocess.run(["git", "add", file_path], cwd=str(self.repo_root), check=True)
            return {"status": "success", "file": file_path, "strategy": strategy}
        except Exception as e:
            return {"status": "error", "message": str(e)}
