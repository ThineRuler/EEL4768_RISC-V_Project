#!/usr/bin/env bash
# Run the phase-2 grader on your submission: the same grade.py, testbenches
# (source/rtl/*_tb.v) and Verilog rule checker (ece552) used for grading.
#
# Usage:
#   scripts/student_test.sh [submission_dir]
#
#   [submission_dir]  directory holding your alu.v, decoder.v, imm.v and rf.v;
#                     defaults to phase_2/submission/
#
# Everything lands in phase_2/output/ -- log.txt (the grader's full output,
# with scores and every failing check) and results.json. Each run replaces the
# previous one.
#
# Needs iverilog, vvp and ece552 on your PATH. Install ece552 once with:
#
#   pip install phase_2/source/python-ece552/
#
# It exits nonzero if any test failed.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The script lives in phase_2/scripts/; everything it reads and writes --
# source/, output/ -- lives one level up. Both are absolute and derived from
# this file's own location, so the script works from any working directory.
PHASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ $# -gt 1 ]]; then
    echo "usage: $0 [submission_dir]" >&2
    exit 1
fi

SUBMISSION_ARG="${1:-${PHASE_DIR}/submission}"
if [[ ! -d "${SUBMISSION_ARG}" ]]; then
    echo "ERROR: submission directory not found: ${SUBMISSION_ARG}" >&2
    exit 1
fi
SUBMISSION_DIR="$(cd "${SUBMISSION_ARG}" && pwd)"

for tool in iverilog vvp ece552 python3; do
    command -v "${tool}" >/dev/null 2>&1 || {
        echo "ERROR: ${tool} not found on PATH." >&2
        echo "iverilog and vvp come from Icarus Verilog (see phase_2/environment.yaml);" >&2
        echo "install ece552 with: pip install ${PHASE_DIR}/source/python-ece552/" >&2
        exit 1
    }
done

OUTPUT_DIR="${PHASE_DIR}/output"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}"
echo "Submission under test: ${SUBMISSION_DIR}"

python3 "${PHASE_DIR}/source/grade.py" "${SUBMISSION_DIR}" \
    --results-dir "${OUTPUT_DIR}" 2>&1 | tee "${OUTPUT_DIR}/log.txt"

# Exit status from the structured results, not from grade.py: it exits 0
# whenever it managed to grade, whatever the score. Only tests worth points
# count, matching the failures grade.py prints; the 0-point presence checks
# are informational.
python3 - "${OUTPUT_DIR}/results.json" <<'PY'
import json, sys
with open(sys.argv[1]) as f:
    data = json.load(f)
tests = data.get("tests")
if not tests:
    sys.exit(1)
sys.exit(1 if any(t.get("status") != "passed" and float(t.get("max_score", 0)) > 0
                  for t in tests) else 0)
PY
