#!/usr/bin/env bash
#
# metrics.sh — portable LOC / function / complexity snapshot
#

MAKEFILE=Makefile
CSV_OUT=metrics.csv
FUNCREG='^[[:space:]]*[A-Za-z_][A-Za-z0-9_[:space:]*]*[(][^;]*[)][[:space:]]*[{][[:space:]]*$'

red=$(tput setaf 1); reset=$(tput sgr0)

printf "%-25s %6s %7s %7s %7s %8s %6s %6s\n" \
  File LOC Funcs AvgFn MaxFn InMK Used CCX
echo "--------------------------------------------------------------------------"
echo "file,loc,funcs,avg_fn,max_fn,in_make,used,ccx" > "$CSV_OUT"

for src in *.c; do
  base=${src%.c}
  loc=$(wc -l < "$src")

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
  END {
      if (funcs==0) print "0 0 0";
      else          printf "%d %.0f %d", funcs, sum/funcs, max
  }' "$src")
EOF

  grep -q "${base}.o" "$MAKEFILE" && inmake=yes || inmake=no

  if [[ -f ${base}.h ]]; then
    grep -R --include=\*.{c,h} "\"${base}.h\"" . >/dev/null && used=yes || used=no
  else
    used=n/a
  fi

  if command -v pmccabe >/dev/null 2>&1; then
    ccx=$(pmccabe "$src" | awk '{s+=$1} END{print (s=="")?0:s}')
  else
    ccx=$(grep -E -o '\b(if|for|while|case)\b' "$src" | wc -l)
  fi

  color=$reset
  [[ $loc   -gt 800 ]] && color=$red
  [[ $ccx   -gt  50 ]] && color=$red
  [[ $maxfn -gt 200 ]] && color=$red

  printf "%s%-25s%s %6d %7d %7.0f %7d %8s %6s %6d\n" \
         "$color" "$src" "$reset" "$loc" "$funcs" "$avgfn" "$maxfn" \
         "$inmake" "$used" "$ccx"

  echo "$src,$loc,$funcs,$avgfn,$maxfn,$inmake,$used,$ccx" >> "$CSV_OUT"
done

echo -e "\nCSV written to $CSV_OUT"

