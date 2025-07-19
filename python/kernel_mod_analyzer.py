#!/usr/bin/env python3
"""
kernel_mod_analyzer.py
Step‑0  : discover & cache the repo’s *initial* commit (BASE0)
Phase‑1 : extract C‑function signatures with Universal‑ctags
          and diff TARGET ↔ BASE0.

Requires:
  • Python ≥ 3.8
  • Git CLI on PATH
  • Universal‑ctags (https://github.com/universal-ctags/ctags) on PATH
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple

# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

CACHE_ROOT = Path.home() / ".cache" / "kernel_mod_ir"
IR_VERSION = "F1_v1"                       # bump if you change schema
BASELINE_FILE = CACHE_ROOT / "baseline.json"


def run(cmd: List[str], *, cwd: Path | None = None) -> str:
    """Run a shell command and return stdout (raise on error)."""
    res = subprocess.run(cmd, cwd=cwd, check=True, text=True, capture_output=True)
    return res.stdout.strip()


def ensure_cache() -> None:
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)


def discover_root_commit(repo: Path) -> str:
    """Return SHA of the root (first) commit."""
    return run(["git", "rev-list", "--max-parents=0", "HEAD"], cwd=repo).split()[0]


def write_baseline_info(root_sha: str) -> None:
    ensure_cache()
    if BASELINE_FILE.exists():
        # keep existing frozen baseline
        return
    data = {
        "baseline_name": "BASE0",
        "commit": root_sha,
        "frozen": True,
    }
    BASELINE_FILE.write_text(json.dumps(data, indent=2))
    print(f"[INFO] Baseline locked to root commit {root_sha}")


# --------------------------------------------------------------------------- #
# Phase‑1 Extractor (Universal‑ctags JSON)
# --------------------------------------------------------------------------- #

def worktree_checkout(repo: Path, commit: str) -> Path:
    """Checkout a commit into a temporary detached worktree (auto‑cleaned)."""
    tmp = Path(tempfile.mkdtemp(prefix=f"ktree_{commit[:8]}_"))
    run(["git", "worktree", "add", "--detach", str(tmp), commit], cwd=repo)
    return tmp


def ctags_extract(worktree: Path) -> List[Dict]:
    """Run ctags and return list of function entries."""
    cmd = [
        "ctags",
        "-R",
        "--languages=C",
        "--kinds-C=fp",             # only functions & prototypes
        "--fields=+neK",            # line number, end line, scope kind
        "--output-format=json",
        "-f", "-"                   # stream to stdout
    ]
    out = run(cmd, cwd=worktree)
    # each JSON object is on its own line
    return [json.loads(line) for line in out.splitlines()
            if line and line.strip().startswith("{")]


def build_f1_ir(tag_entries: List[Dict], commit: str) -> Dict:
    """Transform ctags output into our IR schema."""
    functions = []
    for ent in tag_entries:
        if ent.get("kind") != "function":
            continue
        # A stable ID: SHA1 of "path:name"
        node_id = f"{ent['path']}::{ent['name']}"
        functions.append({
            "id": node_id,
            "name": ent["name"],
            "file": ent["path"],
            "line": ent["line"],
            "end": ent.get("end"),
            "signature": ent.get("signature"),
            "scope": ent.get("scopeKind", "unknown"),
        })
    return {
        "ir_version": IR_VERSION,
        "commit": commit,
        "functions": functions,
    }


def cache_path(commit: str) -> Path:
    return CACHE_ROOT / IR_VERSION / f"{commit}.json"


def extract_or_load(repo: Path, commit: str) -> Dict:
    """Return IR for commit, using cache if available."""
    ensure_cache()
    path = cache_path(commit)
    if path.exists():
        return json.loads(path.read_text())

    wt = worktree_checkout(repo, commit)
    try:
        ir = build_f1_ir(ctags_extract(wt), commit)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(ir))
        print(f"[INFO] cached IR for {commit[:8]} → {path}")
        return ir
    finally:
        # Clean up worktree
        run(["git", "worktree", "remove", "--force", str(wt)], cwd=repo)
        shutil.rmtree(wt, ignore_errors=True)


# --------------------------------------------------------------------------- #
# Phase‑1 Differ
# --------------------------------------------------------------------------- #

def diff_f1(base: Dict, target: Dict) -> Tuple[List, List, List]:
    """
    Return (added, removed, signature_changed)
    where items are node IDs.
    """
    base_map = {f["id"]: f for f in base["functions"]}
    tgt_map = {f["id"]: f for f in target["functions"]}

    added = [fid for fid in tgt_map if fid not in base_map]
    removed = [fid for fid in base_map if fid not in tgt_map]

    signature_changed = []
    for fid in tgt_map.keys() & base_map.keys():
        if tgt_map[fid]["signature"] != base_map[fid]["signature"]:
            signature_changed.append(fid)

    return added, removed, signature_changed


def pretty_diff(added, removed, changed, target_ir):
    """Print a Markdown‑friendly summary to stdout."""
    lookup = {f["id"]: f for f in target_ir["functions"]}
    print("\n### Function‑level delta vs BASE0\n")
    if added:
        print("**Added**")
        for fid in added:
            f = lookup.get(fid, {"name": fid})
            print(f"  • {f['name']} ({f['file']}:{f['line']})")
        print()
    if removed:
        print("**Removed**")
        for fid in removed:
            print(f"  • {fid}")
        print()
    if changed:
        print("**Signature changed**")
        for fid in changed:
            f = lookup.get(fid, {"name": fid})
            print(f"  • {f['name']} ({f['file']}:{f['line']})")
        print()
    if not (added or removed or changed):
        print("No public‑symbol deltas detected 🎉\n")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Step‑0 & Phase‑1 analyzer for a Linux‑kernel module repo")
    parser.add_argument("repo", type=Path,
                        help="Path to the git repository (module source)")
    parser.add_argument("--target", default="HEAD",
                        help="Commit/branch to analyze (default: HEAD)")
    args = parser.parse_args()

    repo = args.repo.resolve()
    if not (repo / ".git").exists():
        sys.exit(f"{repo} is not a git repo")

    # Step 0 – Baseline setup
    root_sha = discover_root_commit(repo)
    write_baseline_info(root_sha)

    # Phase 1 – extract IR for BASE0 and TARGET
    base_ir = extract_or_load(repo, root_sha)
    target_sha = run(["git", "rev-parse", args.target], cwd=repo)
    target_ir = extract_or_load(repo, target_sha)

    added, removed, sig_changed = diff_f1(base_ir, target_ir)
    pretty_diff(added, removed, sig_changed, target_ir)


if __name__ == "__main__":
    main()

