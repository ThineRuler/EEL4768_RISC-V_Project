#!/usr/bin/env bash
# Quick local pass/fail check of a phase-1 submission: the real grader, over
# .data configuration 1 only, assembly programs only.
#
# Usage:
#   scripts/student_test.sh <name> [submission_dir]
#
#   <name>            directory name for this run; everything lands in
#                     phase_1/output/submissions/<name>/ -- summary.txt (the
#                     PASS/FAIL list, plus why each failure failed) and
#                     configuration 1's actual RARS dumps
#   [submission_dir]  directory holding your addition.s, gemm.s, mult.s and
#                     sobel.s; defaults to phase_1/submission/
#
# This runs source/grade.py -- the same script Gradescope runs -- with
# AUTOGRADER_CONFIGS=1 and AUTOGRADER_PARTS=assembly. It exits nonzero if any
# test failed. It does not check your assembler, and the graded run uses two
# more .data configurations: passing here is necessary, not sufficient.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The script lives in phase_1/scripts/; everything it reads and writes --
# source/, output/ -- lives one level up. Both are absolute and derived from
# this file's own location, so the script works from any working directory.
PHASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 <name> [submission_dir]" >&2
    exit 1
fi
# <name> becomes a path segment under output/submissions/ that is then
# rm -rf'd, so an empty or path-bearing name is refused.
case "$1" in
    "" | . | .. | */* | *\\*)
        echo "ERROR: <name> must be a plain directory name (got '$1')" >&2
        exit 1 ;;
esac

SUBMISSION_ARG="${2:-${PHASE_DIR}/submission}"
if [[ ! -d "${SUBMISSION_ARG}" ]]; then
    echo "ERROR: submission directory not found: ${SUBMISSION_ARG}" >&2
    exit 1
fi
SUBMISSION_DIR="$(cd "${SUBMISSION_ARG}" && pwd)"
echo "Submission under test: ${SUBMISSION_DIR}"

OUTPUT_DIR="${PHASE_DIR}/output/submissions/$1"
SUMMARY_FILE="${OUTPUT_DIR}/summary.txt"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}"

TMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/phase1_test.XXXXXX")"
trap 'rm -rf "${TMP_ROOT}"' EXIT

# --- Stage the grading tree --------------------------------------------------
# grade.py reads <AUTOGRADER_ROOT>/submission and writes
# <AUTOGRADER_ROOT>/results/results.json. source/ is COPIED so a run can never
# write into the tree it is grading from.
ROOT="${TMP_ROOT}/root"
mkdir -p "${ROOT}/results" "${ROOT}/submission"
cp -r "${PHASE_DIR}/source" "${ROOT}/source"
if [[ -n "$(ls -A "${SUBMISSION_DIR}")" ]]; then
    cp -r "${SUBMISSION_DIR}/." "${ROOT}/submission/"
fi

# --- Grade -------------------------------------------------------------------
env "AUTOGRADER_ROOT=${ROOT}" \
    "AUTOGRADER_RARS_JAR=${ROOT}/source/rars1_6.jar" \
    "AUTOGRADER_KEEP_OUTPUTS=${OUTPUT_DIR}" \
    "AUTOGRADER_CONFIGS=1" \
    "AUTOGRADER_PARTS=assembly" \
    python3 "${ROOT}/source/grade.py"

RESULTS_JSON="${ROOT}/results/results.json"
if [[ ! -f "${RESULTS_JSON}" ]]; then
    echo "ERROR: the grader produced no results at ${RESULTS_JSON}" >&2
    exit 1
fi

# --- Summarize ---------------------------------------------------------------
# PASS/FAIL comes from each test's status, never from its score: addition is a
# 0-point diagnostic and reads 0.00 / 0.00 either way.
status=0
python3 - "${RESULTS_JSON}" "${SUMMARY_FILE}" <<'PY' || status=$?
import json, sys

with open(sys.argv[1]) as f:
    data = json.load(f)
tests = data.get("tests", [])

lines = ["===== configuration 1 pass/fail ====="]
for t in tests:
    lines.append("  %-4s %-6s %s"
                 % ("PASS" if t.get("status") == "passed" else "FAIL",
                    t.get("number", "?"), t.get("name", "?")))
failed = [t for t in tests if t.get("status") != "passed"]
lines.append("")
lines.append("%d of %d checks passed."
             % (len(tests) - len(failed), len(tests)))
if failed:
    lines.append("Failed: " + ", ".join(t.get("name", "?") for t in failed))
    # Why each failure failed, inline: a summary that only says FAIL is not
    # worth reading.
    for t in failed:
        lines.append("")
        lines.append("=" * 74)
        lines.append("%s  %s" % (t.get("number", "?"), t.get("name", "?")))
        lines.append("=" * 74)
        lines.append(t.get("output", "").rstrip())

text = "\n".join(lines) + "\n"
with open(sys.argv[2], "w") as f:
    f.write(text)
print()
print(text, end="")
sys.exit(1 if failed else 0)
PY

echo "Wrote ${SUMMARY_FILE}"
exit "${status}"
