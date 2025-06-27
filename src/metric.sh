#!/usr/bin/env bash
#
# metrics.sh — gather per-.c metrics to spot big or unused files
#

MAKEFILE=Makefile

printf "%-25s %6s %8s %10s %12s %6s\n" File LOC Funcs In_Make Used CCX
echo "--------------------------------------------------------------------------------"

for src in *.c; do
    base=${src%.c}
    # total lines:
    loc=$(wc -l < "$src")
    # count "foo(...){", i.e. probable function defs:
    funcs=$(grep -E '^[[:space:]]*[A-Za-z_][A-Za-z0-9_[:space:]*]*\([^;]*\)[[:space:]]*\{' "$src" | wc -l)

    # is foo.o in your Makefile?
    grep -q "${base}.o" "$MAKEFILE" && inmake="yes" || inmake="no"
    # is foo.h included anywhere (simple 'used' heuristic)?
    if [[ -f "${base}.h" ]]; then
      grep -R --include=\*.c --include=\*.h "include \"${base}.h\"" . >/dev/null \
        && used="yes" || used="no"
    else
      used="n/a"
    fi
    # crude CCX: count of control‐flow keywords
    ccx=$(grep -E -o '\b(if|for|while|case)\b' "$src" | wc -l)

    printf "%-25s %6d %8d %10s %12s %6d\n" \
      "$src" "$loc" "$funcs" "$inmake" "$used" "$ccx"
done

