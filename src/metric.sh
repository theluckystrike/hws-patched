#!/usr/bin/env bash
#
# metrics.sh — file-level code-health snapshot
#
# Columns:
#   srcLOC   lines in .c
#   hdrLOC   lines in matching .h  (0 if none)
#   Ratio    hdrLOC/srcLOC   (API surface)
#   Funcs    function count
#   AvgFn    average lines per function
#   MaxFn    longest function (lines)
#   CCX      total cyclomatic complexity (pmccabe or lizard; fallback keyword count)
#   maxCCX   worst single function complexity
#   Callers  # of calls to symbols defined in this .c from other .c files
#

set -euo pipefail

MAKEFILE=Makefile
CSV_OUT=metrics.csv
FUNCREG='^[[:space:]]*[A-Za-z_][A-Za-z0-9_[:space:]*]*[(][^;]*[)][[:space:]]*[{][[:space:]]*$'

red=$(tput setaf 1 2>/dev/null || true)
reset=$(tput sgr0 2>/dev/null || true)

printf "%-25s %6s %6s %6s %6s %6s %7s %7s %7s %7s\n" \
  File srcLOC hdrLOC Ratio Funcs AvgFn MaxFn CCX maxCCX Callers
echo "------------------------------------------------------------------------------------------------------"
echo "file,src_loc,hdr_loc,ratio,funcs,avg_fn,max_fn,ccx,max_ccx,callers" > "$CSV_OUT"

# -------------------------------------------------------------------------
# Build simple tag list of all function definitions for caller counting
# -------------------------------------------------------------------------
if command -v ctags &>/dev/null; then
  ctags -x --c-kinds=f *.c > .tags.tmp 2>/dev/null
else
  : > .tags.tmp      # empty file so later tests still work
fi

for src in *.c; do
  base=${src%.c}
  hdr=${base}.h

  srcLOC=$(wc -l < "$src")
  hdrLOC=$([[ -f $hdr ]] && wc -l < "$hdr" || echo 0)
  ratio=$(awk -v s=$srcLOC -v h=$hdrLOC 'BEGIN{printf "%.2f", (s? h/s : 0)}')

  # ---------- function count, avg length, max length ----------
  read funcs avgfn maxfn <<EOF
$(awk -v funcreg="$FUNCREG" '
  BEGIN { funcs=0; sum=0; max=0; inside=0; n=0 }
  {
    if (match($0, funcreg)) { inside=1; n=1; funcs++; next }
    if (inside) {
      n++
      if ($0 ~ /\}/) { inside=0; sum+=n; if (n>max) max=n }
    }
  }
  END{
    if (funcs==0) print "0 0 0";
    else printf "%d %.0f %d", funcs, sum/funcs, max
  }' "$src")
EOF

  # ---------- complexity totals (pmccabe → lizard → keyword) ----------
  if command -v pmccabe &>/dev/null; then
    ccx=$(pmccabe "$src"        | awk '{sum+=$1} END{print sum+0}')
    maxccx=$(pmccabe -f "$src"  | awk 'NF{if($1>m)m=$1} END{print m+0}')
  elif command -v lizard &>/dev/null; then
    read ccx maxccx <<EOF
$(lizard "$src" \
   | sed -n '4,$p' \
   | awk '/^[[:space:]]*[0-9]+/{ccn=int($2); sum+=ccn; if(ccn>mx)mx=ccn} \
          END{print sum+0, mx+0}')
EOF
  else
    ccx=$(grep -E -o '\b(if|for|while|case)\b' "$src" | wc -l)
    maxccx=0
  fi

  # ---------- caller count via ctags ----------
  callers=0
  if [[ -s .tags.tmp ]]; then
    mapfile -t defs < <(grep "[[:space:]]$src\$" .tags.tmp | awk '{print $1}')
    for sym in "${defs[@]}"; do
      cnt=$(grep -R --exclude="$src" -n --include=\*.c -E "\b${sym}[[:space:]]*\(" . \
            | wc -l)
      callers=$((callers + cnt))
    done
  fi

  # ---------- colour rows that exceed soft thresholds ----------
  color=$reset
  (( srcLOC  > 800 )) && color=$red
  (( maxccx  >  25 )) && color=$red
  awk -v r="$ratio" 'BEGIN{exit (r>0.4?0:1)}' && color=$red || true

  printf "%s%-25s%s %6d %6d %6s %6d %6.0f %7d %7d %7d\n" \
    "$color" "$src" "$reset" \
    "$srcLOC" "$hdrLOC" "$ratio" "$funcs" "$avgfn" "$maxfn" \
    "$ccx" "$maxccx" "$callers"

  echo "$src,$srcLOC,$hdrLOC,$ratio,$funcs,$avgfn,$maxfn,$ccx,$maxccx,$callers" \
    >> "$CSV_OUT"
done

rm -f .tags.tmp
echo -e "\nCSV written to $CSV_OUT"

