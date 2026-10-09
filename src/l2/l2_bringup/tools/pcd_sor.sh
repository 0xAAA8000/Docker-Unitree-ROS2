#!/bin/bash
# Remove isolated noise points with CloudCompare's SOR filter (statistical outlier removal).
#
#   pcd_sor.sh IN.pcd OUT.pcd [NEIGHBORS=6] [SIGMA=1.0]
#
# Runs CloudCompare (snap) on the command line and rewrites its output as a plain
# x y z intensity PCD (CloudCompare adds '_' padding fields other tools cannot read).
set -e
IN=$(realpath "$1"); OUT=$(realpath -m "$2"); K=${3:-6}; SIGMA=${4:-1.0}
TOOLS=$(dirname "$(realpath "$0")")
# the snap cannot see /tmp, so work next to the input; -AUTO_SAVE OFF stops intermediate files
TMP="$(dirname "$IN")/.sor_$$.pcd"
trap 'rm -f "$TMP"' EXIT
cloudcompare.CloudCompare -SILENT -AUTO_SAVE OFF -O "$IN" -SOR "$K" "$SIGMA" \
    -C_EXPORT_FMT PCD -SAVE_CLOUDS FILE "$TMP" 2>&1 | grep -v -E 'Gtk-WARNING|Session management' | grep -E 'saved|finished|rror' || true
python3 "$TOOLS/pcd_io.py" "$TMP" "$OUT"
python3 - "$IN" "$OUT" <<'PY'
import sys
def n(p):
    for line in open(p, 'rb'):
        if line.startswith(b'POINTS'): return int(line.split()[1])
a, b = n(sys.argv[1]), n(sys.argv[2])
print(f'{a:,} -> {b:,} points ({(a - b) / a * 100:.1f}% removed): {sys.argv[2]}')
PY
