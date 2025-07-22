#!/usr/bin/env python3
"""
kernel_mod_analyzer.py  (Step‑0, Phase‑1, Phase‑2)

Usage examples
--------------
# Compare HEAD to the initial commit (baseline auto‑discovered)
./kernel_mod_analyzer.py /path/to/repo

# Compare a feature branch tip to baseline
./kernel_mod_analyzer.py /path/to/repo --target my‑branch
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple, Set

# --------------------------------------------------------------------------- #
# Configuration                                                               #
# --------------------------------------------------------------------------- #

CACHE_ROOT = Path.home() / ".cache" / "kernel_mod_ir"

F1_VER = "F1_v1"     # function‑sig IR
CG_VER = "CG_v1"     # call‑graph IR

BASELINE_FILE = CACHE_ROOT / "baseline.json"

# Kern‑subsystem dependency patterns for delta amplification
DEPENDENCY_PATTERNS = {
    "memory_alloc": re.compile(r"\b(km[ac]alloc|kfree|dma_alloc|dma_free)\b"),
    "locking":      re.compile(r"\b(spin_lock|spin_unlock|mutex_lock|mutex_unlock)\b"),
    "workqueue":    re.compile(r"\b(schedule_work|queue_work|flush_work)\b"),
}

INDENT_RE = re.compile(r"^(\s*)([A-Za-z0-9_]+)")

# --------------------------------------------------------------------------- #
# Misc helpers                                                                #
# --------------------------------------------------------------------------- #

def run(cmd: List[str], *, cwd: Path | None = None) -> str:
    res = subprocess.run(cmd, cwd=cwd, check=True, text=True, capture_output=True)
    return res.stdout.strip()

def sha1(text: str) -> str:
    return hashlib.sha1(text.encode()).hexdigest()

def ensure_cache() -> None:
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)

def cache_path(ir_ver: str, commit: str) -> Path:
    return CACHE_ROOT / ir_ver / f"{commit}.json"

# --------------------------------------------------------------------------- #
# Step‑0  (baseline discovery)                                                #
# --------------------------------------------------------------------------- #

def discover_root_commit(repo: Path) -> str:
    return run(["git", "rev-list", "--max-parents=0", "HEAD"], cwd=repo).split()[0]

def write_baseline_info(root_sha: str) -> None:
    ensure_cache()
    if BASELINE_FILE.exists():
        return
    BASELINE_FILE.write_text(json.dumps({
        "baseline_name": "BASE0",
        "commit": root_sha,
        "frozen": True,
    }, indent=2))
    print(f"[INFO] Locked baseline to root commit {root_sha}")

# --------------------------------------------------------------------------- #
# Worktree utility                                                            #
# --------------------------------------------------------------------------- #

def worktree_checkout(repo: Path, commit: str) -> Path:
    tmp = Path(tempfile.mkdtemp(prefix=f"ktree_{commit[:8]}_"))
    run(["git", "worktree", "add", "--detach", str(tmp), commit], cwd=repo)
    return tmp

def cleanup_worktree(repo: Path, wt: Path) -> None:
    try:
        run(["git", "worktree", "remove", "--force", str(wt)], cwd=repo)
    finally:
        shutil.rmtree(wt, ignore_errors=True)

# --------------------------------------------------------------------------- #
# Phase‑1  – function‑signature IR                                            #
# --------------------------------------------------------------------------- #

def ctags_extract(worktree: Path) -> List[Dict]:
    cmd = [
        "ctags", "-R",
        "--languages=C", "--kinds-C=fp", "--fields=+neK+s", "--output-format=json",
        "-f", "-"
    ]
    out = run(cmd, cwd=worktree)
    return [json.loads(l) for l in out.splitlines() if l.startswith("{")]

def build_f1_ir(tag_entries: List[Dict], commit: str) -> Dict:
    fn = []
    for e in tag_entries:
        if e.get("kind") != "function":
            continue
        node_id = f"{e['path']}::{e['name']}"
        fn.append({
            "id":       node_id,
            "name":     e["name"],
            "file":     e["path"],
            "line":     e["line"],
            "signature": e.get("signature"),
            "scope":    e.get("scopeKind", "unknown"),
        })
    return {"ir_version": F1_VER, "commit": commit, "functions": fn}

def extract_or_load_f1(repo: Path, commit: str) -> Dict:
    ensure_cache()
    cp = cache_path(F1_VER, commit)
    if cp.exists():
        return json.loads(cp.read_text())

    wt = worktree_checkout(repo, commit)
    try:
        ir = build_f1_ir(ctags_extract(wt), commit)
        cp.parent.mkdir(parents=True, exist_ok=True)
        cp.write_text(json.dumps(ir))
        print(f"[INFO] cached F1 for {commit[:8]} → {cp.name}")
        return ir
    finally:
        cleanup_worktree(repo, wt)

def diff_f1(base: Dict, tgt: Dict) -> Tuple[List, List, List]:
    bmap = {f["id"]: f for f in base["functions"]}
    tmap = {f["id"]: f for f in tgt["functions"]}
    added   = [fid for fid in tmap if fid not in bmap]
    removed = [fid for fid in bmap if fid not in tmap]
    sigchg  = [fid for fid in tmap.keys() & bmap.keys()
               if tmap[fid]["signature"] != bmap[fid]["signature"]]
    return added, removed, sigchg

def build_alias_map(base: Dict, tgt: Dict) -> Dict[str, str]:
    """
    Map tgt_id -> base_id when function appears to be renamed
    (same file + signature but different name).
    """
    map_by_sig: Dict[Tuple[str,str], str] = {}
    for f in base["functions"]:
        key = (f["file"], f.get("signature"))
        map_by_sig[key] = f["id"]

    alias = {}
    for f in tgt["functions"]:
        key = (f["file"], f.get("signature"))
        b_id = map_by_sig.get(key)
        if b_id and b_id != f["id"]:
            alias[f["id"]] = b_id
    return alias

def pretty_diff_f1(added, removed, changed, tgt_ir):
    lookup = {f["id"]: f for f in tgt_ir["functions"]}
    print("\n### Phase 1 – Function‑level delta vs BASE0\n")
    if added:
        print("**Added:**")
        for fid in added:
            f = lookup.get(fid, {})
            print(f"  • {f.get('name', fid)} ({f.get('file','?')}:{f.get('line','?')})")
        print()
    if removed:
        print("**Removed:**")
        for fid in removed:
            print(f"  • {fid}")
        print()
    if changed:
        print("**Signature changed:**")
        for fid in changed:
            f = lookup.get(fid, {})
            print(f"  • {f.get('name', fid)} ({f.get('file','?')}:{f.get('line','?')})")
        print()
    if not (added or removed or changed):
        print("No public‑symbol deltas detected.\n")

CLANG_BASE_ARGS = ["-fsyntax-only",
                   "-Xclang", "-analyzer-checker=debug.DumpCallGraph"]

# --------------------------------------------------------------------------- #
# Phase-2  – Call-graph extraction with GCC                                   #
# --------------------------------------------------------------------------- #

EDGE_RE     = re.compile(r'^\s*([\w\.\*]+)(?:/\d+)?\s*->\s*([\w\.\*]+)(?:/\d+)?')
LIST_RE = re.compile(r'^\s*(Called by|Calls):\s*(.*)$')
SYMBOL_RE = re.compile(r'([\w\.\*]+)/\d+')

def gcc_cgraph_extract(worktree: Path) -> List[Tuple[str, str]]:
    """
    Build under worktree/src (where the Makefile lives, with your
    -fdump-ipa-cgraph flags) and return all (caller, callee) pairs
    from the generated .ipa-cgraph dumps.
    """
    edges: List[Tuple[str, str]] = []
    current = None

    # 0. Purge any stale dumps
    for old in worktree.rglob("*.cgraph"):
        old.unlink(missing_ok=True)

    # 1. Select build directory
    build_dir = worktree / "src"
    if not (build_dir / "Makefile").exists():
        build_dir = worktree

    # 2. Kick off a full rebuild there; ignore errors
    try:
        run(
            ["make", "-B", f"-j{os.cpu_count() or 8}"],
            cwd=build_dir,
        )
    except subprocess.CalledProcessError:
        pass  # dumps may still have been generated

    # 3. Harvest and clean up the dumps
    for dump in worktree.rglob("*.cgraph"):
        try:
            for line in dump.read_text().splitlines():
                # 1) detect a definition header and remember its name
                hdr = re.match(r'^([\w\.\*]+)(?:/\d+)?\s+\(([\w\.\*]+)\)', line)
                if hdr:
                    # use the *symbol* before the slash as the function name
                    current = hdr.group(1)
                    continue

                # 2) catch explicit arrow edges
                m = EDGE_RE.match(line)
                if m:
                    print('match edges', line)
                    edges.append((m.group(1), m.group(2)))
                    continue

                # 3) catch Called by / Calls lists
                m_lst = LIST_RE.match(line)
                if m_lst and current:
                    kind, rest = m_lst.groups()
                    # find only symbol/ID tokens
                    for sym in SYMBOL_RE.findall(rest):
                        if kind == "Called by":
                            # sym calls current
                            edges.append((sym, current))
                        else:
                            # current calls sym
                            edges.append((current, sym))

        finally:
            dump.unlink(missing_ok=True)

    return edges

def build_cg_ir(edges: List[Tuple[str, str]], commit: str) -> Dict:
    nodes: Set[str] = set()
    for a, b in edges:
        nodes.update([a, b])
    fan_out = Counter(a for a, _ in edges)
    fan_in  = Counter(b for _, b in edges)
    return {
        "ir_version": CG_VER,
        "commit": commit,
        "nodes": sorted(nodes),
        "edges": [list(e) for e in edges],
        "fan_in": fan_in,
        "fan_out": fan_out,
    }

def extract_or_load_cg(repo: Path, commit: str) -> Dict:
    """
    Cache‑aware wrapper around clang_extract_edges.
    """
    ensure_cache()
    cp = cache_path(CG_VER, commit)
    if cp.exists():
        return json.loads(cp.read_text())

    wt = worktree_checkout(repo, commit)
    try:
        edges = gcc_cgraph_extract(wt)
        ir = build_cg_ir(edges, commit)
        cp.parent.mkdir(parents=True, exist_ok=True)
        cp.write_text(json.dumps(ir))
        print(f"[INFO] cached CG for {commit[:8]} → {cp.name}")
        return ir
    finally:
        cleanup_worktree(repo, wt)

# --------------------------------------------------------------------------- #
# Phase‑2  – Differ                                                           #
# --------------------------------------------------------------------------- #

Edge   = Tuple[str,str]
EdgeSet= Set[Edge]

def rewrite_graph(ir: Dict, alias: Dict[str,str]) -> Tuple[Set[str], EdgeSet]:
    """Replace node IDs per alias map; return (nodes, edges) as sets."""
    def ali(x): return alias.get(x,x)
    nodes = {ali(n) for n in ir["nodes"]}
    edges = {(ali(a), ali(b)) for a,b in map(tuple, ir["edges"])}
    return nodes, edges

def diff_cg(base_ir: Dict, tgt_ir: Dict, alias: Dict[str,str]) -> Dict:
    b_nodes, b_edges = rewrite_graph(base_ir, alias)
    t_nodes, t_edges = rewrite_graph(tgt_ir, alias)

    added_nodes   = t_nodes - b_nodes
    removed_nodes = b_nodes - t_nodes
    added_edges   = t_edges - b_edges
    removed_edges = b_edges - t_edges

    # fan‑out delta for existing nodes
    fan_out_delta = {}
    for n in t_nodes & b_nodes:
        bfo = base_ir["fan_out"].get(n, 0)
        tfo = tgt_ir["fan_out"].get(n, 0)
        if tfo != bfo:
            fan_out_delta[n] = (bfo, tfo)

    return {
        "added_nodes": list(added_nodes),
        "removed_nodes": list(removed_nodes),
        "added_edges": [list(e) for e in added_edges],
        "removed_edges": [list(e) for e in removed_edges],
        "fan_out_delta": fan_out_delta,
    }

def amplify_dependency_deltas(added_edges: EdgeSet) -> Dict[str,List[Edge]]:
    """Return {class:[edges]} for first‑ever subsystem dependencies."""
    flagged: Dict[str,List[Edge]] = defaultdict(list)
    for caller, callee in added_edges:
        for cls, pat in DEPENDENCY_PATTERNS.items():
            if pat.search(callee):
                flagged[cls].append((caller, callee))
    return flagged

def pretty_diff_cg(result: Dict, dep_flags: Dict[str,List[Edge]]) -> None:
    print("### Phase 2 – Call‑graph delta vs BASE0\n")
    print(f"**Nodes:** +{len(result['added_nodes'])}  –{len(result['removed_nodes'])}")
    print(f"**Edges:** +{len(result['added_edges'])}  –{len(result['removed_edges'])}\n")

    if result["fan_out_delta"]:
        print("**Fan‑out changes (caller → #callees):**")
        for n,(old,new) in result["fan_out_delta"].items():
            print(f"  • {n}: {old} → {new}")
        print()

    if dep_flags:
        print("**First‑ever calls into key subsystems:**")
        for cls, edges in dep_flags.items():
            print(f"  *{cls}*")
            for c,a in edges:
                print(f"    {c} → {a}")
        print()
    if not any([result["added_nodes"], result["removed_nodes"],
                result["added_edges"], result["removed_edges"]]):
        print("Call‑graph identical to baseline.\n")

# --------------------------------------------------------------------------- #
# CLI                                                                         #
# --------------------------------------------------------------------------- #

def main() -> None:
    p = argparse.ArgumentParser(description="Kernel‑module analyzer (Step‑0 + P1 + P2)")
    p.add_argument("repo", type=Path, help="Path to git repo")
    p.add_argument("--target",   default="HEAD",     help="Commit/branch to analyze")
    p.add_argument("--baseline", default="baseline", help="Commit/branch to diff against (default: 'baseline')")
    p.add_argument("--clean-ir", action="store_true",
                   help="Delete cached IR for baseline and target commits before analysis")
    args = p.parse_args()

    repo = args.repo.resolve()
    if not (repo / ".git").exists():
        sys.exit(f"{repo} is not a git repository")

    # Normalize shas
    target_sha = run(["git", "rev-parse", args.target], cwd=repo)
    # Resolve baseline / target SHAs up‑front
    baseline_sha = run(["git", "rev-parse", args.baseline], cwd=repo)
    target_sha   = run(["git", "rev-parse", args.target],   cwd=repo)

    # Optional: purge cached IR so we start fresh
    if args.clean_ir:
        for sha in (baseline_sha, target_sha):
            for ver in (F1_VER, CG_VER):
                cache_path(ver, sha).unlink(missing_ok=True)

    # Step‑0
    # root_sha = discover_root_commit(repo)

    write_baseline_info(baseline_sha)
    # Phase‑1
    base_f1 = extract_or_load_f1(repo, baseline_sha)
    tgt_f1  = extract_or_load_f1(repo, target_sha)
    added, removed, sigchg = diff_f1(base_f1, tgt_f1)
    pretty_diff_f1(added, removed, sigchg, tgt_f1)

    # Alias map for renames
    alias = build_alias_map(base_f1, tgt_f1)

    # Phase‑2
    base_cg = extract_or_load_cg(repo, baseline_sha)
    tgt_cg  = extract_or_load_cg(repo, target_sha)
    cg_res  = diff_cg(base_cg, tgt_cg, alias)

    # Amplification heuristic
    added_edges_set = {tuple(e) for e in cg_res["added_edges"]}
    dep_flags = amplify_dependency_deltas(added_edges_set)

    pretty_diff_cg(cg_res, dep_flags)


if __name__ == "__main__":
    main()

