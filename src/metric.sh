#!/usr/bin/env bash
#
# metrics.sh — file-level code-health dashboard
# Works on Arch with `ctags` + `pip install --user lizard`
#

set -euo pipefail

MAKEFILE=Makefile
CSV_OUT=metrics.csv
FUNCREG='^[[:space:]]*[A-Za-z_][A-Za-z0-9_[:space:]*]*[(][^;]*[)][[:space:]]*[{][[:space:]]*$'

# Colours only if we’re on a TTY
if [[ -t 1 ]]; then
  red=$(tput setaf 1); reset=$(tput sgr0)
else
  red=''; reset=''
fi

printf "%-25s %6s %6s %6s %6s %6s %7s %7s %7s %7s %7s\n" \
  File srcLOC hdrLOC Ratio Funcs AvgFn MaxFn CCX maxCCX Callers
echo "-------------------------------------------------------------------------------------------------------------"
echo "file,src_loc,hdr_loc,ratio,funcs,avg_fn,max_fn,ccx,max_ccx,callers" > "$CSV_OUT"

# ------------------------------------------------------------------------
# Tags for caller count (ignore if ctags missing)
# ------------------------------------------------------------------------
if command -v ctags &>/dev/null; then
  ctags -x --c-kinds=f *.c > .tags.tmp 2>/dev/null
else
  : > .tags.tmp
fi

for src in *.c; do
  base=${src%.c}
  hdr=${base}.h

  srcLOC=$(wc -l < "$src")
  hdrLOC=$([[ -f $hdr ]] && wc -l < "$hdr" || echo 0)
  ratio=$(awk -v s=$srcLOC -v h=$hdrLOC 'BEGIN{printf "%.2f", (s? h/s : 0)}')

  # -------- function metrics --------
  read funcs avgfn maxfn <<EOF
$(awk -v funcreg="$FUNCREG" '
  BEGIN{funcs=sum=max=inside=n=0}
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

  # -------- complexity totals (pmccabe → lizard → fallback) --------
  if command -v pmccabe &>/dev/null; then
    ccx=$(pmccabe "$src"        | awk '{s+=$1} END{print s+0}')
    maxccx=$(pmccabe -f "$src"  | awk 'NF && $1>m{m=$1} END{print m+0}')
  elif command -v lizard &>/dev/null; then
    read ccx maxccx <<EOF
$(lizard "$src" \
   | sed -n '4,$p' \
   | awk '/@'"$src"'$/ { ccn=int($2); sum+=ccn; if(ccn>mx) mx=ccn } \
          END{print sum+0, mx+0}')
EOF
  else
    ccx=$(grep -E -o '\b(if|for|while|case)\b' "$src" | wc -l)
    maxccx=0
  fi

  # -------- caller count --------
  callers=0
  if [[ -s .tags.tmp ]]; then
    mapfile -t defs < <(grep "[[:space:]]$src\$" .tags.tmp | awk '{print $1}')
    for sym in "${defs[@]}"; do
      callers=$((callers + $(grep -R --exclude="$src" --include=\*.c -E "\b${sym}[[:space:]]*\(" . | wc -l)))
    done
  fi

  # -------- red flag? --------
  color=$reset
  (( srcLOC  > 800 )) && color=$red
  (( maxccx  >  25 )) && color=$red
  awk -v r="$ratio" 'BEGIN{exit r>0.4?0:1}' && color=$red || true

  printf "%s%-25s%s %6d %6d %6s %6d %6.0f %7d %7d %7d %7d\n" \
    "$color" "$src" "$reset" \
    "$srcLOC" "$hdrLOC" "$ratio" "$funcs" "$avgfn" "$maxfn" \
    "$ccx" "$maxccx" "$callers"

  echo "$src,$srcLOC,$hdrLOC,$ratio,$funcs,$avgfn,$maxfn,$ccx,$maxccx,$callers" >> "$CSV_OUT"
done

rm -f .tags.tmp
echo -e "\nCSV written to $CSV_OUT"

