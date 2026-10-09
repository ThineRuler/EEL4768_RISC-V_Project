#!/usr/bin/env bash
# Phase 4 setup check: "will my processor run in the autograder's setup?"
#
# Usage:
#   scripts/check_design.sh <name> [submission_dir]
#
#   <name>            directory name for this run; everything lands in
#                     phase_4/output/submissions/<name>/
#   [submission_dir]  design to check; defaults to source/reference/
#
# THIS SCRIPT DOES NOT GRADE AND DOES NOT CHECK CORRECTNESS. It answers one
# question: does your design instantiate, compile, reset, retire instructions
# and halt under the same harness shape the autograder uses? It runs a
# ~10-instruction program and never compares a single computed value against
# an expected one. A design can pass this and still be completely wrong.
#
# Use scripts/student_test.sh (the recorded ~24k-vector trace) to find out
# whether your processor is actually CORRECT. Run this one first: if your
# design can't even get through this, the trace check's output will be noise.
#
# Why this exists: the autograder decides pass/fail by parsing the text its
# testbench prints. Debug output left in your own RTL ($display, $write,
# $monitor) is interleaved into that text and corrupts it, which silently
# deletes instructions from the trace the grader sees -- a correct processor
# scoring zero. The bulk trace self-check compares signals inside Verilog and
# never parses text, so it cannot detect this. This script can.
#
# It needs iverilog and nothing else -- no python, no ece552. If iverilog is
# not on your PATH, either put it there or set IVERILOG to the binary:
#
#   IVERILOG=/opt/iverilog/bin/iverilog scripts/check_design.sh mine
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PHASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -f "${SCRIPT_DIR}/hart_smoke_tb.v" && -f "${SCRIPT_DIR}/smoke_program.hex" ]]; then
    SMOKE_TB="${SCRIPT_DIR}/hart_smoke_tb.v"
    SMOKE_HEX="${SCRIPT_DIR}/smoke_program.hex"
else
    TRACES="${PHASE_DIR}/source/traces"
    SMOKE_TB="${TRACES}/rtl/hart_smoke_tb.v"
    SMOKE_HEX="${TRACES}/vectors/smoke_program.hex"
fi

# The smoke program is 11 words: 10 retire, 1 is skipped by a taken branch.
# Used only to word the "ended early" error -- a count mismatch on a run
# that reached the end is never a failure here, or this would become a
# correctness check by the back door.
EXPECTED_RETIRED=10

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "usage: $0 <name> [submission_dir]" >&2
    exit 1
fi
case "$1" in
    "" | . | .. | */* | *\\*)
        echo "ERROR: <name> must be a plain directory name (got '$1')" >&2
        exit 1 ;;
esac

OUTPUT_DIR="${PHASE_DIR}/output/submissions/$1"
SUBMISSION_ARG="${2:-${PHASE_DIR}/source/reference}"

if [[ ! -d "${SUBMISSION_ARG}" ]]; then
    echo "ERROR: no such directory: ${SUBMISSION_ARG}" >&2
    exit 1
fi
SUBMISSION="$(cd "${SUBMISSION_ARG}" && pwd)"

for f in "${SMOKE_TB}" "${SMOKE_HEX}"; do
    [[ -f "${f}" ]] || { echo "ERROR: missing ${f}" >&2; exit 1; }
done

IVERILOG="${IVERILOG:-iverilog}"
if ! command -v "${IVERILOG}" >/dev/null 2>&1 && command -v conda >/dev/null 2>&1; then
    CONDA_BASE="$(conda info --base 2>/dev/null | tail -1)"
    CONDA_BIN="${CONDA_BASE}/envs/${IVERILOG_CONDA_ENV:-iverilog}/bin"
    [[ -x "${CONDA_BIN}/iverilog" ]] && export PATH="${CONDA_BIN}:${PATH}"
fi
if ! command -v "${IVERILOG}" >/dev/null 2>&1; then
    echo "ERROR: iverilog not found (tried '${IVERILOG}')." >&2
    echo "Put it on your PATH, or set IVERILOG to the binary." >&2
    exit 1
fi

mapfile -t SOURCES < <(find "${SUBMISSION}" \
    \( -name '*.v' -o -name '*.sv' \) \
    -not -path '*/.git/*' -not -path '*/__MACOSX/*' \
    -not -path '*/obj_dir/*' | sort)
if [[ ${#SOURCES[@]} -eq 0 ]]; then
    echo "ERROR: no .v or .sv files under ${SUBMISSION}" >&2
    exit 1
fi
WORK="${OUTPUT_DIR}/check_design"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${WORK}"

# Collected as we go, printed together at the end. Any entry means the
# design will NOT be gradable.
problems=()

# ---------------------------------------------------------------------------
# 1. `include -- a hard reject, mirroring source/grade.py's
#    find_include_directive(), which rejects a submission outright BEFORE
#    compiling it. Worth its own check so the message says what's wrong
#    rather than surfacing later as a confusing compile error.
# ---------------------------------------------------------------------------
include_hits="$(grep -n '^[[:space:]]*`include' "${SOURCES[@]}" 2>/dev/null || true)"
if [[ -n "${include_hits}" ]]; then
    problems+=("ERROR: \`include is not allowed; the autograder rejects it before compiling. Remove:")
    while IFS= read -r line; do
        problems+=("  ${line#"${SUBMISSION}/"}")
    done <<< "${include_hits}"
fi

# ---------------------------------------------------------------------------
# 2. Print statements in the source. Not a failure on its own (legal
#    Verilog, and it may sit behind a condition that never fires); shown
#    only alongside step 5's failure, so the student can find the lines.
# ---------------------------------------------------------------------------
debug_hits="$(grep -nE '\$(display|write|monitor|strobe|fdisplay|fwrite)\b' "${SOURCES[@]}" 2>/dev/null || true)"

# Stop before compiling if `include is present: that is exactly what the
# grader does, and compiling anyway would give a misleading second error.
if [[ ${#problems[@]} -eq 0 ]]; then

# ---------------------------------------------------------------------------
# 3. Compile, the same way the grader does. -s hart_smoke_tb restricts
#    elaboration to this testbench's own module tree, exactly as grade.py's
#    compile_hart_tb() uses -s hart_tb -- so a testbench of your own sitting
#    in the same folder does not get elaborated alongside it.
# ---------------------------------------------------------------------------
if ! "${IVERILOG}" -g2005 -s hart_smoke_tb -o "${WORK}/sim" \
        "${SMOKE_TB}" "${SOURCES[@]}" > "${WORK}/build.log" 2>&1; then
    problems+=("ERROR: your design did not compile. iverilog said:")
    while IFS= read -r line; do
        problems+=("  ${line}")
    done < <(head -25 "${WORK}/build.log")
else

    # -----------------------------------------------------------------------
    # 4. Run the ~10-instruction program.
    # -----------------------------------------------------------------------
    cp "${SMOKE_HEX}" "${WORK}/program.mem"
    # vvp rather than executing ./sim directly: iverilog's output carries a
    # #!/.../vvp shebang that has no equivalent on Windows, even under Git
    # Bash. Same reasoning as student_test.sh.
    if ! (cd "${WORK}" && vvp sim > output.txt 2>&1); then
        problems+=("ERROR: the simulation could not run to completion. Output:")
        while IFS= read -r line; do
            problems+=("  ${line}")
        done < <(head -25 "${WORK}/output.txt")
    else

        # -------------------------------------------------------------------
        # 5. Read the verdict out of the output.
        #
        #    Every line our testbench prints starts with "SMOKE:". Anything
        #    else on stdout came from the submission's own RTL -- which is
        #    precisely what corrupts the graded trace. This catches the case
        #    the grep in step 2 cannot: output produced by a module that only
        #    prints under some condition, or through a construct the grep
        #    does not match.
        # -------------------------------------------------------------------
        # Normalise before analysing: iverilog emits CRLF on Windows, and it
        # writes its own diagnostics ($readmemh range warnings, the "$finish
        # called at" notice) to stdout too. Those name hart_smoke_tb.v --
        # they are the simulator talking about OUR file, not the submission
        # printing something, so they must not count as contamination.
        tr -d '\r' < "${WORK}/output.txt" \
            | grep -vF 'hart_smoke_tb.v' > "${WORK}/output.clean.txt"

        stray="$(grep -vE '^SMOKE:' "${WORK}/output.clean.txt" | grep -vE '^[[:space:]]*$' || true)"
        retired="$(sed -n 's/^SMOKE: cycles=[0-9]* retired=\([0-9]*\) .*/\1/p' "${WORK}/output.clean.txt" | tail -1)"
        # A $finish in the submission kills the run before our trailer line
        # is ever printed, so fall back to counting the per-retirement lines.
        # Without this, a design that retired several instructions and THEN
        # died gets told it "never retired a single instruction" -- wrong,
        # and it buries the real cause.
        if [[ -z "${retired}" ]]; then
            retired="$(grep -c '^SMOKE:   retired #' "${WORK}/output.clean.txt" || true)"
        fi

        if [[ -n "${stray}" ]]; then
            problems+=("ERROR: your design printed its own output, which corrupts the trace the autograder parses. Lines from your code:")
            while IFS= read -r line; do
                problems+=("  ${line}")
            done < <(printf '%s\n' "${stray}" | head -15)
            if [[ -n "${debug_hits}" ]]; then
                problems+=("Print statements in your source:")
                while IFS= read -r line; do
                    problems+=("  ${line#"${SUBMISSION}/"}")
                done <<< "${debug_hits}"
            fi
        fi

        if ! grep -q "^SMOKE: END$" "${WORK}/output.clean.txt"; then
            problems+=("ERROR: the simulation ended early; a \$finish or \$stop in your RTL is the usual cause.")
        elif grep -q "^SMOKE: DID NOT HALT$" "${WORK}/output.clean.txt"; then
            problems+=("ERROR: your processor never halted; the final ebreak must drive o_retire_halt high when it retires.")
        fi

        if [[ -z "${retired}" || "${retired}" -eq 0 ]]; then
            problems+=("ERROR: your processor never retired an instruction; o_retire_valid was never high.")
        elif [[ "${retired}" -lt "${EXPECTED_RETIRED}" ]] \
             && ! grep -q "^SMOKE: END$" "${WORK}/output.clean.txt"; then
            problems+=("ERROR: only ${retired} of ${EXPECTED_RETIRED} instructions retired before the run ended.")
        fi
    fi
fi
fi

if [[ ${#problems[@]} -gt 0 ]]; then
    printf '%s\n' "${problems[@]}"
    exit 1
fi
echo "Your design compiled successfully."
