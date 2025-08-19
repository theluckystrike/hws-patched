#!/usr/bin/env python3
import argparse, os, re
from pathlib import Path

# Match object-like defines:   #define NAME ...
DEFINE_RE = re.compile(r'^\s*#\s*define\s+([A-Za-z_][A-Za-z0-9_]*)\b(?!\s*\()', re.M)

# File types to scan
SOURCE_EXTS = {".c", ".h", ".cc", ".cpp", ".hpp", ".hh"}

def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return ""

def collect_defines(header: Path):
    text = read_text(header)
    names = set(DEFINE_RE.findall(text))
    return names

def iter_source_files(root: Path, skip: Path):
    for dirpath, dirnames, filenames in os.walk(root):
        # Skip VCS / build dirs
        base = os.path.basename(dirpath)
        if base in {".git", ".svn", ".hg", "build", "out"}:
            continue
        for fn in filenames:
            p = Path(dirpath) / fn
            if p.suffix.lower() in SOURCE_EXTS and p.resolve() != skip.resolve():
                yield p

def find_usages(root: Path, header: Path, names):
    # Precompile regex per name for whole-word matches
    regexes = {n: re.compile(r'\b' + re.escape(n) + r'\b') for n in names}
    uses = {n: [] for n in names}

    for p in iter_source_files(root, header):
        text = read_text(p)
        if not text:
            continue
        for n, rx in regexes.items():
            if rx.search(text):
                uses[n].append(str(p))

    return uses

def main():
    ap = argparse.ArgumentParser(description="Report #defines only present in the given header (unused elsewhere).")
    ap.add_argument("--header", required=True, type=Path, help="Path to header file (e.g., hws.h)")
    ap.add_argument("--root", default=Path("."), type=Path, help="Root of source tree to scan (default: .)")
    ap.add_argument("--ignore", nargs="*", default=[], help="Define names to ignore (space-separated)")
    args = ap.parse_args()

    header = args.header.resolve()
    root = args.root.resolve()
    ignore = set(args.ignore or [])

    if not header.exists():
        raise SystemExit(f"Header not found: {header}")

    defines = collect_defines(header) - ignore
    if not defines:
        print(f"No object-like #defines found in {header}")
        return

    uses = find_usages(root, header, defines)

    unused = sorted([n for n, files in uses.items() if not files])
    used    = {n: files for n, files in uses.items() if files}

    print(f"# Header: {header}")
    print(f"# Root:   {root}")
    print(f"# Total defines found: {len(defines)}")
    print()

    if unused:
        print("## Unused (only defined in header):")
        for n in unused:
            print(f"  - {n}")
        print()

    print("## Used elsewhere:")
    if not used:
        print("  (none)")
    else:
        for n, files in sorted(used.items()):
            # show up to 5 paths per define
            shown = files[:5]
            more = "" if len(files) <= 5 else f" (+{len(files)-5} more)"
            print(f"  - {n}:")
            for f in shown:
                print(f"      {f}")
            if more:
                print(f"      {more}")

if __name__ == "__main__":
    main()

