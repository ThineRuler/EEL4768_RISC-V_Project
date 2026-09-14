import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# ---------------------------------------------------------------------------
# Project-2-only local grading CLI -- no Docker/Gradescope required.
#
# Same grading logic as ../docker/submit.py (itself a trimmed fork of the
# parent 552-grading-client repo's run_project2_local()) -- only the
# entrypoint differs: this is a plain command-line tool a TA runs directly
# against a submission directory on their own machine, and it prints a
# human-readable summary in addition to writing results.json (since there's
# no Gradescope UI here to render that file). See ../docker/README.md and
# the parent repo's CLAUDE.md for the full design history behind the
# grading logic itself.
#
# Requires `iverilog` and `ece552` (pip install ./python-ece552/, from this
# folder) on PATH -- see README.md.
# ---------------------------------------------------------------------------

SOURCE_DIR = Path(__file__).resolve().parent
PROJECT2_TB_DIR = SOURCE_DIR / "rtl"
PROJECT2_ALU_OPS = ["ADD", "SUB", "SLL", "SLT", "SLTU", "XOR", "SRL", "SRA", "OR", "AND"]
PROJECT2_DECODER_GROUPS = ["OP", "OPIMM", "LOAD", "STORE", "BRANCH", "UPPER", "JUMP", "SYSTEM", "ILLEGAL"]
PROJECT2_IMM_FORMATS = ["I", "S", "B", "U", "J"]
# rf_no_bypass_tb.v / rf_bypass_tb.v print this and nothing else when a check
# fails, and print nothing at all when they pass.
PROJECT2_RF_FAIL_MARKER = "TEST FAILED:"
# A broken rf fails most of its 31 registers twice over; showing every line
# buries the first, which is the one that explains the bug.
PROJECT2_RF_MAX_FAILURES = 12
# The files this project actually grades. Everything else a submission
# happens to contain is compiled alongside them but is not rule-checked --
# see the ece552 validate call in grade_project2().
PROJECT2_GRADED_FILES = ["alu.v", "decoder.v", "imm.v", "rf.v"]

# UCF grading policy (differs from the parent repo's per-test point values,
# e.g. ALU ops at 2pts/imm at 15pts/rf at 5pts): every graded testbench is
# worth an equal share of a fixed total, however many testbenches there are,
# rather than each carrying its own fixed weight. A testbench scored as more
# than one test subdivides its own share and no one else's -- rf's 1/3
# no-bypass, 2/3 bypass. See assign_even_scores().
PROJECT2_TOTAL_POINTS = 3


def write_results(results_dir, payload):
    os.makedirs(results_dir, exist_ok=True)
    results_path = os.path.join(results_dir, "results.json")
    with open(results_path, "w") as results_file:
        json.dump(payload, results_file, indent=2)
    return results_path


def error_result(message):
    return {"score": 0, "output": message}


def collect_submission_files(submission_dir, work_dir, extensions=(".v", ".txt")):
    """Copy every file matching `extensions` from submission_dir into
    work_dir, preserving relative paths. project2 needs both .v (the
    graded RTL) and .txt (project2.txt's presence check)."""
    copied_files = []
    for root, _, filelist in os.walk(submission_dir):
        for name in filelist:
            if name.endswith(extensions):
                src = os.path.join(root, name)
                rel = os.path.relpath(src, submission_dir)
                dst = os.path.join(work_dir, rel)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy2(src, dst)
                copied_files.append(dst)
    return copied_files


INCLUDE_DIRECTIVE = re.compile(r"^\s*`include\b", re.MULTILINE)


def find_include_directive(files):
    """Return the first file (a Path) among `files` containing a Verilog
    `` `include `` preprocessor directive, or None if none do.

    `` `include `` is invisible to both the ece552 validator and
    iverilog/pyslang's own compile step (it's resolved before either ever
    parses the result), so a submission containing e.g.
    `` `include "/etc/hostname" `` gets that file's content spliced
    directly into the source text, and both tools' compile diagnostics
    echo the surrounding source lines verbatim into grading output.
    Rejected outright before any submission file ever reaches iverilog or
    ece552."""
    for f in files:
        try:
            text = Path(f).read_text(errors="replace")
        except (OSError, UnicodeDecodeError):
            continue
        if INCLUDE_DIRECTIVE.search(text):
            return f
    return None


ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")


def strip_ansi(text):
    """Strip ANSI/VT100 escape sequences -- recent Icarus Verilog versions
    colorize diagnostics by default even when piped through subprocess."""
    return ANSI_ESCAPE.sub("", text)


def run_cmd(args, cwd=None, timeout=60):
    """Run `args` (an argv list -- no shell) with an enforced,
    unbypassable timeout. Takes an explicit argv list rather than a
    `bash -c "..."` string (unlike the parent repo's/../docker/'s
    run_cmd) so this works unmodified on a TA's own machine, Windows
    included -- no `bash`/POSIX-shell dependency, no shell-quoting or
    `*.v`-glob-expansion pitfalls translating a Windows path through a
    shell command string, and iverilog's compiled output isn't directly
    executable on Windows anyway (it has to be run via `vvp`, not
    `./binary` -- see compile_and_run_project2_tb).

    Since nothing here is invoked through a shell chain (`&&`, pipes),
    each call spawns exactly one direct child process, so a plain kill()
    on timeout is sufficient -- no grandchild-process-group concern.

    Never raises: returns a subprocess.CompletedProcess either way, with
    returncode=-9 and an explanatory stderr on timeout.

    Decodes output as UTF-8 explicitly (errors="replace") rather than
    `text=True`'s platform-default encoding, and forces PYTHONIOENCODING
    so a Python-based child (ece552) doesn't fall back to escaping its own
    UTF-8 box-drawing diagnostic characters as literal "\\uXXXX" text --
    confirmed happening on Windows, where the default console code page
    isn't UTF-8."""
    env = {**os.environ, "PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1"}
    proc = subprocess.Popen(
        args, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        encoding="utf-8", errors="replace", env=env,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
        return subprocess.CompletedProcess(
            args, proc.returncode, strip_ansi(stdout), strip_ansi(stderr)
        )
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()  # reap the now-dead process
        return subprocess.CompletedProcess(
            args, -9, "", f"Command timed out after {timeout}s and was killed."
        )


VALIDATE_PARSE_ERROR_MARKER = "not running compliance checks"


def validate_error_message(stderr):
    """`ece552 validate` exits 1 for two different situations that can't be
    told apart by exit code alone: a real syntax/parse error, or a
    rule/style violation on otherwise-valid, compiling code. validate.py
    prints an explicit "...not running compliance checks" line only in the
    genuine parse-error case, so that marker picks the accurate
    message."""
    if VALIDATE_PARSE_ERROR_MARKER in stderr:
        return f"Your code does not compile. Please fix compilation errors before resubmitting.\n{stderr}"
    return f"Your submission violates one or more Verilog style/synthesizability rules (see ../rules on the course site). This is not a compile error.\n{stderr}"


# Where a diagnostic happened: both formats `ece552 validate` emits carry a
# `<path>.v:<line>:<col>` location -- slang's parse errors inline, prettyerr's
# compliance violations inside a box header.
DIAGNOSTIC_LOCATION = re.compile(r"([^\s:'\"\[\]]+\.v):\d+:\d+")


def validate_graded_files(work_dir, present):
    """Rule-check the graded files and return ({filename: message}, fatal).

    Checked together, because they are not independent -- decoder.v
    instantiates imm, so imm.v has to be in the same compilation unit or
    decoder.v is rejected for a module it can plainly see.

    Blame is assigned one file per pass, to the file the FIRST diagnostic
    points at, which is then dropped and the rest re-checked. Everything
    after that first diagnostic may be fallout rather than fault: a defect
    that leaves a construct unterminated -- `module oops( ;` -- swallows the
    files that follow it, and slang duly reports errors against decoder.v
    and rf.v for a defect that lives in alu.v. Blaming every file named in
    one pass would charge three testbenches for one file's bug. Blaming only
    the first, then re-checking, lets a file that is genuinely broken in its
    own right resurface on the next pass, and one that was only downstream
    of the mess come back clean.

    `fatal` is non-None only when the checker itself failed to run; a
    submission's own defects always come back as entries in the dict."""
    broken = {}
    while True:
        good = [name for name in present if name not in broken]
        if not good:
            return broken, None
        result = run_cmd(["ece552", "validate", *[str(work_dir / name) for name in good]],
                         cwd=work_dir, timeout=30)
        if result.returncode == 0:
            return broken, None
        if result.returncode != 1:
            return broken, f"An error occurred while validating your Verilog files.\n{result.stderr}"

        message = validate_error_message(result.stderr)
        first = DIAGNOSTIC_LOCATION.search(result.stderr)
        culprit = Path(first.group(1)).name if first else None
        if culprit in good:
            broken[culprit] = message
        else:
            # Nothing in the output points at a file we are still checking,
            # so nothing can be cleared: whatever is left answers for it.
            for name in good:
                broken[name] = message
            return broken, None


def compile_and_run_project2_tb(work_dir, tb_name, out_name, broken_files=None):
    """Compile the submission's *.v (already collected into work_dir)
    against a vendored project2 testbench and run it. Returns
    (stdout, None) on success or (None, error_message) on compile/
    simulation failure.

    -s <top> restricts elaboration to just the grading testbench's module
    tree. Without it, iverilog elaborates every top-level module it finds,
    so a submission with an extra top module (a student's own testbench,
    or one rigged to $finish immediately) can run concurrently with the
    real testbench and cause grading to miss real failures.

    Run via `vvp <out_name>` rather than `./<out_name>` -- iverilog's -o
    output is directly executable via a `#!/.../vvp` shebang on Unix, but
    that doesn't work on Windows, where it has to be handed to vvp
    explicitly. vvp works identically on both, so this one form covers
    both without an OS check."""
    top = tb_name[:-2]
    broken_files = broken_files or {}

    # A file that failed validation fails its OWN testbench and no others:
    # alu.v's syntax error is not evidence about rf.v. So the testbench whose
    # DUT is broken reports that, and every other testbench is compiled with
    # the broken file left out -- otherwise one unparseable file would fail
    # all four compiles and the submission would score 0 for a defect in a
    # quarter of it.
    own_file = tb_name.replace("_tb.v", ".v")
    if own_file in broken_files:
        return None, broken_files[own_file]

    # Drop any submitted file that declares this testbench's own top module.
    # Students who zip the handout tree intact ship our testbenches back to
    # us; adding ours alongside theirs declares the module twice, and
    # iverilog rejects the duplicate before a single test can run. The
    # submission is not at fault for that, so drop the file, not the
    # submission. Matched on the module name rather than the filename, so a
    # renamed copy is caught too. Every other extra .v is kept -- a
    # submission is free to split its design across helper modules.
    duplicate_top = re.compile(rf"^\s*module\s+{re.escape(top)}\b", re.M)
    v_files = sorted(
        str(path) for path in work_dir.glob("*.v")
        if path.name not in broken_files
        and not duplicate_top.search(path.read_text(errors="replace"))
    )
    compile_result = run_cmd(
        ["iverilog", "-s", top, "-o", out_name, *v_files, str(PROJECT2_TB_DIR / tb_name)],
        cwd=work_dir, timeout=60,
    )
    if compile_result.returncode != 0:
        return None, f"Compilation failed:\n{compile_result.stderr}"

    out_path = work_dir / out_name
    try:
        sim_result = run_cmd(["vvp", out_name], cwd=work_dir, timeout=60)
        if sim_result.returncode != 0:
            return None, f"Simulation failed:\n{sim_result.stderr}"
    finally:
        if out_path.exists():
            os.remove(out_path)
    return sim_result.stdout, None


def grouped_testbench_result(trace, groups):
    """(failed_groups, excerpts) for a testbench that reports one
    `[GROUP FAILURE]` marker per named group in `groups` (alu_tb.v's ten
    ops, decoder_tb.v's nine instruction classes), each preceded by a
    `--- GROUP Tests ---` header. `failed_groups` is empty when every
    group passed. `excerpts` holds, for each failing group, the trace
    slice between its header and its failure marker (or the whole trace
    if either marker is missing) -- shown in the all-or-nothing test's
    output so a TA/student can see exactly which group(s) broke even
    though the testbench is scored as one unit."""
    failed_groups = [g for g in groups if f"[{g} FAILURE]" in trace]
    excerpts = []
    for g in failed_groups:
        start = trace.find(f"--- {g} Tests ---")
        end = trace.find(f"[{g} FAILURE]")
        excerpts.append(trace[start:end] if start != -1 and end != -1 else trace)
    return failed_groups, excerpts


def assign_even_scores(tests, total_points=PROJECT2_TOTAL_POINTS):
    """Split `total_points` evenly across every test in `tests` that
    doesn't already carry a max_score -- the zero-weight file-presence
    checks set their own max_score=0 upfront (see grade_project2()) and
    are left untouched here. Each share is rounded to 4 decimal places,
    and the last scored test absorbs whatever rounding remainder is left
    so the sum of every max_score always equals total_points exactly (not
    total_points +/- a few tests' worth of rounding drift).

    Purely count-driven: the share is total_points / len(unscored),
    recomputed fresh every call from however many test dicts happen to be
    in `tests` at that moment. There's no hardcoded testbench count
    anywhere, so adding or removing a testbench's grading block in
    grade_project2() (one test dict per testbench -- see the comment
    there) changes every remaining testbench's share automatically, with
    no point-value constant to update by hand."""
    unscored = [t for t in tests if "max_score" not in t]
    if not unscored:
        return
    # A test's optional "weight" is how many testbench-shares it is worth,
    # defaulting to a whole one. It exists so a single testbench can be
    # scored as more than one test without that testbench quietly taking a
    # bigger bite of the total than its neighbours: rf_tb.v's two
    # configurations weigh 1/3 and 2/3, so rf still costs exactly one share
    # however it is subdivided. Popped rather than read, so it stays out of
    # the Gradescope-shaped payload.
    weights = [t.pop("weight", 1) for t in unscored]
    per_share = total_points / sum(weights)
    running_total = 0.0
    for idx, (test, weight) in enumerate(zip(unscored, weights)):
        share = (round(total_points - running_total, 4) if idx == len(unscored) - 1
                 else round(per_share * weight, 4))
        test["max_score"] = share
        test["score"] = share if test["status"] == "passed" else 0
        running_total += share


def check_tools():
    missing = [tool for tool in ("iverilog", "vvp", "ece552") if shutil.which(tool) is None]
    if missing:
        print(f"Error: required tool(s) not found on PATH: {', '.join(missing)}")
        print("See README.md for setup instructions (install iverilog, then")
        print("`pip install ./python-ece552/` from this folder).")
        sys.exit(1)


def grade_project2(submission_dir):
    """Run every project2 sub-test against `submission_dir`. Returns a
    Gradescope-shaped results payload: either {"score": 0, "output": ...}
    (a hard failure before any sub-test could run) or {"tests": [...]}."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        work_dir = Path(tmp_dir)

        copied_files = collect_submission_files(submission_dir, tmp_dir)
        if not copied_files:
            return error_result(f"No submission files found in {submission_dir}.")

        include_offender = find_include_directive(copied_files)
        if include_offender:
            return error_result(
                f"Your submission contains a Verilog `include directive "
                f"({Path(include_offender).name}), which is not allowed."
            )

        # Verilog rule checker (ece552 validate). A file that fails it is
        # charged to the one testbench that depends on it rather than to the
        # whole submission: see validate_graded_files() for how failures are
        # attributed, and compile_and_run_project2_tb() for how the rest of
        # the submission is still compiled and graded around them.
        #
        # Scoped to the four graded files, NOT every .v in the zip. A student
        # who submits the handout tree intact ships our own testbenches too,
        # and a testbench is non-synthesizable by nature ($random, $display,
        # behavioural stimulus) -- validating those zeroed submissions whose
        # actual RTL was clean, for the crime of including our code.
        #
        # Run without -w (warnings-as-errors). The checker's job here is to
        # reject code that isn't synthesizable RTL, not to hold a submission
        # to a lint standard the handout never set: -w promoted advice like
        # "place parentheses around the '&' expression" into a zero, on code
        # that compiles and passes every test. Genuine parse errors and rule
        # violations still fail, since those are errors with or without -w.
        present = [name for name in PROJECT2_GRADED_FILES if (work_dir / name).exists()]
        if not present:
            return error_result(
                "None of the graded files (" + ", ".join(PROJECT2_GRADED_FILES)
                + f") were found in {submission_dir}.")
        broken_files, validate_failure = validate_graded_files(work_dir, present)
        if validate_failure:
            return error_result(validate_failure)

        tests = []

        # Zero-weight informational required-file checks.
        for idx, filename in enumerate(["alu.v", "imm.v", "rf.v", "decoder.v", "project2.txt"]):
            present = (work_dir / filename).exists()
            tests.append({
                "number": f"0.{idx}",
                "name": f"{filename} present",
                "max_score": 0,
                "score": 0,
                "status": "passed" if present else "failed",
                "output": f"{filename} found." if present else f"Please submit {filename} (filename must match)!",
            })

        # ALU: one compile+run, one all-or-nothing test -- any failing op
        # anywhere in alu_tb.v zeroes the whole ALU share (assigned evenly
        # alongside imm/rf below by assign_even_scores()), it doesn't
        # partially credit individual ops. Every failing op's excerpt is
        # still included in the output so a TA/student can see exactly
        # what broke.
        alu_trace, alu_error = compile_and_run_project2_tb(work_dir, "alu_tb.v", "alu_out", broken_files)
        alu_test = {"number": "1", "name": "ALU accuracy"}
        if alu_error:
            alu_test["status"] = "failed"
            alu_test["output"] = f"ALU tests failed.\n{alu_error}"
        else:
            failed_ops, excerpts = grouped_testbench_result(alu_trace, PROJECT2_ALU_OPS)
            if failed_ops:
                alu_test["status"] = "failed"
                alu_test["output"] = f"ALU tests failed ({', '.join(failed_ops)}).\n" + "\n".join(excerpts)
            else:
                alu_test["status"] = "passed"
                alu_test["output"] = "Test passed."
        tests.append(alu_test)

        # Decoder: same one-compile-run, one all-or-nothing-test shape as
        # ALU above, just with decoder_tb.v's nine instruction-class groups
        # instead of ALU's ten ops. Submission file: decoder.v (module
        # `decoder`, matching decoder_tb.v's `decoder dut (...)`).
        decoder_trace, decoder_error = compile_and_run_project2_tb(work_dir, "decoder_tb.v", "decoder_out", broken_files)
        decoder_test = {"number": "2", "name": "Decoder accuracy"}
        if decoder_error:
            decoder_test["status"] = "failed"
            decoder_test["output"] = f"Decoder tests failed.\n{decoder_error}"
        else:
            failed_groups, excerpts = grouped_testbench_result(decoder_trace, PROJECT2_DECODER_GROUPS)
            if failed_groups:
                decoder_test["status"] = "failed"
                decoder_test["output"] = f"Decoder tests failed ({', '.join(failed_groups)}).\n" + "\n".join(excerpts)
            else:
                decoder_test["status"] = "passed"
                decoder_test["output"] = "Test passed."
        tests.append(decoder_test)

        # imm: same grouped, all-or-nothing shape as ALU/decoder above --
        # imm_tb.v groups its checks by instruction format (I/S/B/U/J; R has
        # no immediate and is a documented don't-care, so it isn't tested).
        imm_trace, imm_error = compile_and_run_project2_tb(work_dir, "imm_tb.v", "imm_out", broken_files)
        imm_test = {"number": "3", "name": "imm accuracy"}
        if imm_error:
            imm_test["status"] = "failed"
            imm_test["output"] = f"imm tests failed.\n{imm_error}"
        else:
            failed_formats, excerpts = grouped_testbench_result(imm_trace, PROJECT2_IMM_FORMATS)
            if failed_formats:
                imm_test["status"] = "failed"
                imm_test["output"] = f"imm tests failed ({', '.join(failed_formats)}).\n" + "\n".join(excerpts)
            else:
                imm_test["status"] = "passed"
                imm_test["output"] = "Test passed."
        tests.append(imm_test)

        # rf: TWO runs, one per testbench. The spec requires the submission's
        # rf.v to be correct at both BYPASS_EN settings, and each testbench
        # instantiates it at one of them -- rf_no_bypass_tb.v as `rf #(0)`,
        # rf_bypass_tb.v as `rf #(1)` -- so each run scores its own test and a
        # failure in one cannot be confused for a failure in the other.
        #
        # These testbenches are silent on success: every check prints
        # "TEST FAILED: ..." and nothing else, so the verdict is simply
        # whether that marker appears. There are no group markers to parse,
        # and the printed lines are already the diagnosis.
        #
        # The split is uneven on purpose (UCF policy): no-bypass is a third
        # of rf's share, bypass two thirds, the harder half carrying more.
        for number, tb_name, label, weight in (
            ("4", "rf_no_bypass_tb.v", "no bypass", 1 / 3),
            ("5", "rf_bypass_tb.v", "bypass", 2 / 3),
        ):
            # Name kept inside the summary table's 24-column name field.
            rf_test = {"number": number, "name": f"rf accuracy ({label})",
                       "weight": weight}
            rf_trace, rf_error = compile_and_run_project2_tb(
                work_dir, tb_name, f"rf_{number}_out", broken_files)
            if rf_error:
                rf_test["status"] = "failed"
                rf_test["output"] = f"rf tests failed.\n{rf_error}"
            elif PROJECT2_RF_FAIL_MARKER in rf_trace:
                failures = [line for line in rf_trace.splitlines()
                            if PROJECT2_RF_FAIL_MARKER in line]
                rf_test["status"] = "failed"
                rf_test["output"] = (
                    f"rf BYPASS_EN={'1' if label == 'bypass' else '0'} tests "
                    f"failed ({len(failures)} check(s)).\n"
                    + "\n".join(failures[:PROJECT2_RF_MAX_FAILURES]))
            else:
                rf_test["status"] = "passed"
                rf_test["output"] = "Test passed."
            tests.append(rf_test)

        # To grade an additional testbench later: compile+run it with
        # compile_and_run_project2_tb() and append one all-or-nothing test
        # dict for it here (max_score/score unset, no "weight" -- that
        # defaults to one whole testbench share), following the alu_test/
        # imm_test/rf-loop pattern above. assign_even_scores() below picks
        # up the new dict automatically and every testbench's share shrinks
        # accordingly -- no point-value constant to edit anywhere.
        assign_even_scores(tests)
        return {"tests": tests}


def print_summary(payload):
    if "score" in payload:
        print(f"\nFAILED (0 points)\n{payload['output']}")
        return

    total_score = sum(t["score"] for t in payload["tests"])
    total_max = sum(t["max_score"] for t in payload["tests"])
    print(f"\n{'#':<5}{'test':<24}{'status':<10}{'score':>13}")
    print("-" * 52)
    for t in payload["tests"]:
        print(f"{t['number']:<5}{t['name']:<24}{t['status']:<10}{t['score']:>6.4g}/{t['max_score']:<6.4g}")
    print("-" * 52)
    print(f"TOTAL{'':<19}{'':<10}{total_score:>6.4g}/{total_max:<6.4g}")

    for t in payload["tests"]:
        if t["status"] == "failed" and t["max_score"] > 0:
            print(f"\n--- {t['name']} output ---\n{t['output']}")


def main():
    # ece552's diagnostics use Unicode box-drawing characters, which crash
    # a plain print() on Windows's default (non-UTF-8) console code page.
    # reconfigure() is a no-op in effect (replace on genuinely unencodable
    # chars, never a hard crash) everywhere this doesn't matter.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")

    parser = argparse.ArgumentParser(description="Grade a project2 submission (alu.v/decoder.v/imm.v/rf.v) locally.")
    parser.add_argument(
        "submission_dir", nargs="?", default="submission",
        help="Directory containing the student's .v/.txt files (default: ./submission)",
    )
    parser.add_argument(
        "--results-dir", default="results",
        help="Directory to write results.json into (default: ./results)",
    )
    args = parser.parse_args()

    check_tools()

    submission_dir = Path(args.submission_dir).resolve()
    if not submission_dir.is_dir():
        print(f"Error: submission directory not found: {submission_dir}")
        sys.exit(1)

    payload = grade_project2(submission_dir)
    results_path = write_results(args.results_dir, payload)
    print(f"Wrote {results_path}")
    print_summary(payload)


if __name__ == "__main__":
    main()
