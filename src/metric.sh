#!/usr/bin/env bash
#
# metrics.sh — portable LOC / function / complexity snapshot
#

MAKEFILE=Makefile
CSV_OUT=metrics.csv
FUNCREG='^[[:space:]]*[A-Za-z_][A-Za-z0-9_[:space:]*]*[(][^;]*[)][[:space:]]*[{][[:space:]]*$'
red=$(tput setaf 1) ; reset=$(tput sgr0)
printf "%-25s %6s %6s %6s %6s %6s %7s %7s %7s %7s\n" \
  File srcLOC hdrLOC Ratio Funcs AvgFn MaxFn CCX maxCCX Callers
echo "------------------------------------------------------------------------------------------------------"
echo "file,src_loc,hdr_loc,ratio,funcs,avg_fn,max_fn,ccx,max_ccx,callers" > "$CSV_OUT"

#-------------------------------------------------------------------------------
# Build a quick tag list (function name ↦ defining file)
#-------------------------------------------------------------------------------
if command -v ctags >/dev/null 2>&1; then
  ctags -x --c-kinds=f *.c > .tags.tmp 2>/dev/null
fi

for src in *.c; do
  base=${src%.c}
  hdr="${base}.h"

  srcLOC=$(wc -l < "$src")
  if [[ -f $hdr ]]; then
    hdrLOC=$(wc -l < "$hdr")
  else
    hdrLOC=0
  fi
  ratio=$(awk -v s=$srcLOC -v h=$hdrLOC 'BEGIN{ if(s==0) print 0; else printf "%.2f", h/s }')

  #---------------- function counts / avg / max (portable awk) ---------------
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

  #---------------- total CCX & max CCX per-function --------------------------
  if command -v pmccabe &>/dev/null; then
    ccx=$(pmccabe "$src"      | awk '{s+=$1} END{print (s?"":0)+s}')
    maxccx=$(pmccabe -f "$src" | awk 'NF{if($1>m)m=$1} END{print (m?"":0)+m}')
  elif command -v lizard &>/dev/null; then
    # Lizard prints a header + one line per function.  
    # We skip first 3 lines, then grab the 2nd column (CCN) of each
    read ccx maxccx <<EOF
$(lizard "$src" \
   | sed -n '4,$p' \
   | awk '{ ccn=$2; sum+=ccn; if(ccn>m)m=ccn } END{print (sum?"":0)+sum, (m?"":0)+m}')
EOF
  else
    # fallback: keyword count, no per-fn max
    ccx=$(grep -E -o '\b(if|for|while|case)\b' "$src" | wc -l)
    maxccx=0
  fi
  #---------------- caller count (who uses our functions?) --------------------
  callers=0
  if [[ -s .tags.tmp ]]; then
    # get all function symbols defined in this .c
    mapfile -t funcs_defined < <(grep "[[:space:]]$src\$" .tags.tmp | awk '{print $1}')
    for sym in "${funcs_defined[@]}"; do
      # count references in other .c files
      cnt=$(grep -R --exclude="$src" -n --include=\*.c -E "\b${sym}[[:space:]]*\(" . | wc -l)
      callers=$((callers + cnt))
    done
  else
    # fallback: none
    callers=0
  fi

  #----------------- colouring for big bad things ----------------------------
  color=$reset
  [[ $srcLOC  -gt 800 ]] && color=$red
  [[ $maxccx  -gt  25 ]] && color=$red
  [[ $ratio   > 0.40 ]] && color=$red     # lots of API surface
  printf "%s%-25s%s %6d %6d %6.2f %6d %6.0f %6d %7d %7d %7d\n" \
    "$color" "$src" "$reset" \
    "$srcLOC" "$hdrLOC" "$ratio" "$funcs" "$avgfn" "$maxfn" \
    "$ccx" "$maxccx" "$callers"

  echo "$src,$srcLOC,$hdrLOC,$ratio,$funcs,$avgfn,$maxfn,$ccx,$maxccx,$callers" >> "$CSV_OUT"
done

rm -f .tags.tmp
echo -e "\nCSV written to $CSV_OUT"
