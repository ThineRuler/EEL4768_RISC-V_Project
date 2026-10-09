#!/usr/bin/env bash
# Phase 5 setup check: "will my processor run in the autograder's setup?"
#
# Usage:
#   scripts/check_design.sh <name> <submission_dir>
#
#   <name>            directory name for this run; everything lands in
#                     phase_5/output/submissions/<name>/
#   <submission_dir>  the directory that holds your .v files
#
# THIS SCRIPT DOES NOT GRADE AND DOES NOT CHECK CORRECTNESS. It answers one
# question: does your design compile against the phase 5 harness (hart with
# the five cache parameters and the line-wide memory ports), reset, retire
# instructions and halt? It runs a 10-instruction program and never compares
# a single computed value against an expected one. A design can pass this
# and still be completely wrong.
#
# To find out whether your processor is actually CORRECT, run
# traces/random_program.hex in your own testbench and compare its data
# memory with traces/random_dmem.hex. Run this check first: if your design
# can't even get through this, the memory check's output will be noise.
#
# Besides compiling and running, it reports two things the autograder
# rejects before it runs any test: a preprocessor directive that can hide
# code from the style validator, and a system task ($display, $finish, ...),
# which the style validator rejects.
#
# It needs iverilog and nothing else: no python, no ece552. If iverilog is
# not on your PATH, either put it there or set IVERILOG to the binary:
#
#   IVERILOG=/opt/iverilog/bin/iverilog scripts/check_design.sh mine /path/to/your/verilog
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PHASE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SMOKE_TB="${SCRIPT_DIR}/hart_smoke_tb.v"
SMOKE_HEX="${SCRIPT_DIR}/smoke_program.hex"

# The smoke program is 10 words, all of which retire. Used only to word the
# "ended early" error: a count mismatch on a run that reached the end is
# never a failure here, or this would become a correctness check by the
# back door.
EXPECTED_RETIRED=10

if [[ $# -ne 2 ]]; then
    echo "usage: $0 <name> <submission_dir>" >&2
    exit 1
fi
case "$1" in
    "" | . | .. | */* | *\\*)
        echo "ERROR: <name> must be a plain directory name (got '$1')" >&2
        exit 1 ;;
esac

OUTPUT_DIR="${PHASE_DIR}/output/submissions/$1"
SUBMISSION_ARG="$2"

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

# The top-level .v files of the submission (no subdirectories, no .sv): the
# same set the grader compiles.
mapfile -t SOURCES < <(find "${SUBMISSION}" -maxdepth 1 -type f -name '*.v' | sort)
if [[ ${#SOURCES[@]} -eq 0 ]]; then
    echo "ERROR: no .v files at the top level of ${SUBMISSION}" >&2
    exit 1
fi
WORK="${OUTPUT_DIR}/check_design"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${WORK}"

# Collected as we go, printed together at the end. Any entry means the
# design will NOT be gradable.
problems=()

# ---------------------------------------------------------------------------
# 1. Directives that can hide code from the style validator: a hard reject,
#    mirroring the autograder, which rejects a submission BEFORE compiling
#    it. The match is textual, so a directive inside a comment counts too.
#    `ifdef RISCV_FORMAL (from the skeleton) is the one allowed form.
# ---------------------------------------------------------------------------
DIRECTIVE='`(include|ifndef|elsif|ifdef)\b'
directive_hits=()
for f in "${SOURCES[@]}"; do
    while IFS= read -r hit; do
        # Drop each allowed `ifdef RISCV_FORMAL, then see if anything is left.
        rest="$(sed -E 's/`ifdef[[:space:]]+RISCV_FORMAL([^A-Za-z0-9_$]|$)/\1/g' <<< "${hit#*:}")"
        if grep -qE "${DIRECTIVE}" <<< "${rest}"; then
            directive_hits+=("  ${f#"${SUBMISSION}/"}:${hit}")
        fi
    done < <(grep -nE "${DIRECTIVE}" "${f}" 2>/dev/null || true)
done
if [[ ${#directive_hits[@]} -gt 0 ]]; then
    problems+=("ERROR: \`include, \`ifndef, \`elsif and any \`ifdef other than \`ifdef RISCV_FORMAL are not allowed, even in a comment; the autograder rejects the file before compiling. Remove:")
    problems+=("${directive_hits[@]}")
fi

# ---------------------------------------------------------------------------
# 2. System tasks. The autograder runs the style validator (ece552 validate)
#    before any test, and it rejects every system task, so one of these
#    anywhere in your files scores zero. Reported outright, whether or not
#    it ever prints anything in the run below.
# ---------------------------------------------------------------------------
task_hits="$(grep -nE '\$(display|write|monitor|strobe|fdisplay|fwrite|finish|stop|readmemh|readmemb|fopen)\b' "${SOURCES[@]}" /dev/null 2>/dev/null || true)"
if [[ -n "${task_hits}" ]]; then
    problems+=("ERROR: system tasks are not allowed; the style validator rejects them, so the submission scores zero. Remove:")
    while IFS= read -r line; do
        problems+=("  ${line#"${SUBMISSION}/"}")
    done <<< "${task_hits}"
fi

# Stop before compiling if a directive is present: that is exactly what the
# grader does, and compiling anyway would give a misleading second error.
if [[ ${#directive_hits[@]} -eq 0 ]]; then

# ---------------------------------------------------------------------------
# 3. Compile, the same way the grader does. -s hart_smoke_tb restricts
#    elaboration to this testbench's own module tree, exactly as the grader
#    does with its testbench, so a testbench of your own sitting in the same
#    folder does not get elaborated alongside it. A hart without one of the
#    five cache parameters, or without one of the memory ports, fails here:
#    iverilog reports "parameter `X` not found" or "port ``x'' is not a
#    port of dut" as an error.
# ---------------------------------------------------------------------------
if ! "${IVERILOG}" -g2005 -s hart_smoke_tb -o "${WORK}/sim" \
        "${SMOKE_TB}" "${SOURCES[@]}" > "${WORK}/build.log" 2>&1; then
    problems+=("ERROR: your design did not compile. iverilog said:")
    while IFS= read -r line; do
        problems+=("  ${line}")
    done < <(head -25 "${WORK}/build.log")
else

    # -----------------------------------------------------------------------
    # 4. Run the 10-instruction program. -n: a $stop ends the run instead of
    #    waiting at an interactive prompt.
    # -----------------------------------------------------------------------
    # vvp rather than executing ./sim directly: iverilog's output carries a
    # #!/.../vvp shebang that has no equivalent on Windows, even under Git
    # Bash.
    if ! (cd "${WORK}" && vvp -n sim "+program=${SMOKE_HEX}" > output.txt 2>&1); then
        problems+=("ERROR: the simulation could not run to completion. Output:")
        while IFS= read -r line; do
            problems+=("  ${line}")
        done < <(head -25 "${WORK}/output.txt")
    else

        # -------------------------------------------------------------------
        # 5. Read the verdict out of the output.
        #
        #    Every line our testbench prints starts with "SMOKE:". Anything
        #    else on stdout came from the submission's own RTL.
        # -------------------------------------------------------------------
        # Normalise before analysing: iverilog emits CRLF on Windows, and it
        # writes its own diagnostics ($readmemh range warnings, the "$finish
        # called at" notice) to stdout too. Those name hart_smoke_tb.v:
        # they are the simulator talking about OUR file, not the submission
        # printing something, so they must not count as stray output.
        tr -d '\r' < "${WORK}/output.txt" \
            | grep -vF 'hart_smoke_tb.v' > "${WORK}/output.clean.txt"

        stray="$(grep -vE '^SMOKE:' "${WORK}/output.clean.txt" | grep -vE '^[[:space:]]*$' || true)"
        retired="$(sed -n 's/^SMOKE: cycles=[0-9]* retired=\([0-9]*\) .*/\1/p' "${WORK}/output.clean.txt" | tail -1)"
        # A $finish in the submission kills the run before our summary line
        # is ever printed, so fall back to counting the per-retirement lines.
        # Without this, a design that retired several instructions and THEN
        # died gets told it "never retired a single instruction": wrong,
        # and it buries the real cause.
        if [[ -z "${retired}" ]]; then
            retired="$(grep -c '^SMOKE:   retired #' "${WORK}/output.clean.txt" || true)"
        fi

        if [[ -n "${stray}" ]]; then
            problems+=("ERROR: your design printed its own output. Lines from your code:")
            while IFS= read -r line; do
                problems+=("  ${line}")
            done < <(printf '%s\n' "${stray}" | head -15)
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
