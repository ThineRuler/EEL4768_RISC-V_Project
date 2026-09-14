#!/usr/bin/env python3
"""Gradescope autograder for the RISC-V compiler-architecture project, phase 1.

Runs two independent parts against the student's submission:

  Part 1 (Assembly)  - the student's own gemm.s / mult.s / sobel.s are each run
                       headless in RARS under THREE .data configurations (9
                       subtests) and their result-memory dumps are compared
                       against the reference dumps. The .data is spliced in
                       from source/configs/, so a .text that hardcodes the
                       shipped skeleton's answer fails configs 2 and 3.
  Part 2 (Assembler) - the student's assembler.py is run over the reference
                       .s programs in source/reference/, once per .data
                       configuration, and each of its four output files is
                       compared against the reference solution. The .data is
                       spliced in from source/configs/ by the same rule the
                       assembly half uses, so an assembler that special-cases
                       the shipped skeleton's numbers fails configs 2 and 3.
                       This includes the hidden all_instructions test.

The answer key under source/expected/ is grouped by configuration --
expected/{assembly,assembler}/<program>/cfg<N>.* -- and is regenerated from
source/configs/ and source/reference/ by source/gen_expected.py.

AUTOGRADER_CONFIGS restricts a run to a subset of the configurations, which is
how run_test.sh gets a quick configuration-1 pass/fail check out of this very
grader instead of a second, drifting copy of it.

Design rules that the rest of this file exists to uphold:

  * The answer key is read into memory ONCE, before any student code runs, and
    every comparison is made against those in-memory copies. Student code that
    overwrites the expected files on disk therefore cannot change its own grade.
  * Student code is never told a path from which the answer key is reachable.
    It is run from a scratch directory holding nothing but its own copy of the
    assembler and the one .s program under test.
  * Child output is captured to a file and only a bounded slice is ever read
    into memory, so a student who prints gigabytes cannot OOM the grader.
  * <AUTOGRADER_ROOT>/results/results.json is always written, atomically, even
    when the run dies unexpectedly, and partial credit already earned survives.
"""

import json
import os
import pwd
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from fractions import Fraction

try:
    import resource
except ImportError:  # non-POSIX; the rlimit hardening is simply skipped
    resource = None

# --------------------------------------------------------------------------
# Paths
# --------------------------------------------------------------------------

# Everything hangs off one root so the grader can also be run natively for
# testing:  AUTOGRADER_ROOT=/path/to/fake bash run_autograder
# Gradescope leaves the variable unset, so the container resolves to /autograder.
AUTOGRADER_ROOT = os.environ.get("AUTOGRADER_ROOT", "/autograder")

SOURCE_DIR = os.path.join(AUTOGRADER_ROOT, "source")
SUBMISSION_DIR = os.path.join(AUTOGRADER_ROOT, "submission")
RESULTS_DIR = os.path.join(AUTOGRADER_ROOT, "results")
RESULTS_JSON = os.path.join(RESULTS_DIR, "results.json")

# The image mirrors the jar OUTSIDE the 0700 source tree (root-owned 0444),
# because a java child running as the student uid cannot read
# /autograder/source.
#
# The env var is a PREFERENCE, not a promise: run_autograder defaults it to
# /opt/rars/rars1_6.jar, which exists in the packaged image but not on a
# developer machine. Picking the first candidate that actually EXISTS means the
# container uses the readable mirror while a native run silently uses the
# in-source copy -- rather than failing every assembly test because a path was
# exported for an environment we are not in.
def _resolve_rars_jar():
    candidates = [os.environ.get("AUTOGRADER_RARS_JAR"),
                  os.path.join(SOURCE_DIR, "rars1_6.jar")]
    candidates = [c for c in candidates if c]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    return candidates[0]      # keep the preferred path for the error message


RARS_JAR = _resolve_rars_jar()
# The reference .s programs the assembler half is run on, and the reference
# assembler that generated the answer key. Both are part of the key, so the
# whole directory is removed from disk once the key is in memory.
#
# Its internal shape is NOT part of the contract: the programs are located by
# file name at any depth (see find_reference_program), so they may sit flat in
# reference/ or nested under reference/assembly/. source/gen_expected.py
# resolves them the same way, so the two cannot disagree about which file is
# the reference.
REFERENCE_DIR = os.path.join(SOURCE_DIR, "reference")
EXPECTED_DIR = os.path.join(SOURCE_DIR, "expected")
CONFIGS_DIR = os.path.join(SOURCE_DIR, "configs")
MAX_CYCLES_DIR = os.path.join(SOURCE_DIR, "max_cycles")


# Every per-program, per-configuration tree in phase 1 nests the same way:
# the half that needs it first, then the program, then the configuration as
# the leaf file -- matching configs/<program>/<N>.data.
#
#   expected/assembly/gemm/cfg2.txt            the RARS dump
#   expected/assembler/gemm/cfg2.hex.txt       one of four assembler outputs
#   max_cycles/gemm/cfg2.txt                   the instruction budget
#
# max_cycles has no assembly/assembler level because only the assembly half
# runs anything; an assembler has no instruction budget to exceed.
def expected_assembly_path(program, config):
    """The RARS dump one program must produce under one configuration."""
    return os.path.join(EXPECTED_DIR, "assembly", program,
                        "cfg%d.txt" % config)


def expected_assembler_path(program, filename):
    """One of the four files an assembler must produce for one program."""
    return os.path.join(EXPECTED_DIR, "assembler", program, filename)


def max_cycles_path(program, config):
    """The instruction budget for one program under one configuration."""
    return os.path.join(MAX_CYCLES_DIR, program, "cfg%d.txt" % config)

# Opt-in debugging aid for LOCAL runs: when this points at a directory, the
# files each test PRODUCED are copied there before the scratch tree is deleted
# -- the student assembler's gen/ outputs and the RARS memory dumps, not the
# inputs staged in to produce them. Gradescope never sets it -- run_autograder
# does not export it and it is not on ENV_ALLOWLIST -- so in production this
# stays None and nothing is copied.
KEEP_OUTPUTS_DIR = os.environ.get("AUTOGRADER_KEEP_OUTPUTS")

# --------------------------------------------------------------------------
# Student-user contract.
#
# THE DEFAULTS BELOW ARE LOAD-BEARING IN PRODUCTION. DO NOT REMOVE THEM.
#
# Packaging is a Gradescope ZIP upload. setup.sh creates the
# `student` account (uid/gid 1001) at build time, running as root -- but a zip
# autograder has no ENV mechanism, so setup.sh cannot export anything to run
# time. In production all four AUTOGRADER_STUDENT_* variables are therefore
# UNSET and this code runs entirely on the defaults below. Nothing else supplies
# these values.
#
# So the env lookups are the override path (useful for testing and for any
# future image that does export them); the defaults are the real production
# configuration. They must stay byte-identical to the identity setup.sh creates
# -- change one, change both, or the sandbox drops student code to an identity
# that does not exist and Privsep falls back to a bare numeric drop.
#
# AUTOGRADER_RARS_JAR is the one variable that IS set at run time, exported by
# run_autograder, because the in-source jar sits inside the 0700 tree the
# dropped child cannot read. _resolve_rars_jar() above treats it as a preference
# and picks the first candidate that actually EXISTS, which is what keeps native
# runs working when that mirror path is absent.
# --------------------------------------------------------------------------

STUDENT_USER = os.environ.get("AUTOGRADER_STUDENT_USER", "student")
STUDENT_HOME = os.environ.get("AUTOGRADER_STUDENT_HOME", "/home/student")


def _env_id(name, default):
    try:
        value = int(os.environ.get(name, default))
        return value if value > 0 else -1
    except (TypeError, ValueError):
        return -1


STUDENT_UID = _env_id("AUTOGRADER_STUDENT_UID", 1001)
STUDENT_GID = _env_id("AUTOGRADER_STUDENT_GID", 1001)

# --------------------------------------------------------------------------
# Point values -- RETUNE HERE, AND ONLY HERE.
#
# Everything below is derived, never repeated. No point value is written twice:
# per-config assembly values come from POINTS_ASSEMBLY[program] /
# len(CONFIGS), each assembler file weight comes from
# POINTS_ASSEMBLER[test] times that file's share, each half is the sum of its
# parts, and the import-time asserts at the end of this block re-check the
# arithmetic. A retune that stops adding to 3 crashes on import instead of
# silently shipping.
# --------------------------------------------------------------------------

ASSEMBLY_TESTS = ["gemm", "mult", "sobel", "addition"]

# all_instructions is assembler-only (there is no student all_instructions.s)
# and stays LAST: it is the hidden test. Like every other test it is graded
# under all three .data configurations.
ASSEMBLER_TESTS = ["gemm", "mult", "sobel", "addition", "all_instructions"]

# BOTH halves are graded under three .data configurations: the .data is a
# fixed skeleton students must not change, so a .text -- or an assembler --
# that hardcodes the answer for the shipped skeleton fails configs 2 and 3.
ALL_CONFIGS = [1, 2, 3]


def _selected_configs():
    """The configurations this run grades. Normally all of them.

    AUTOGRADER_CONFIGS restricts the run to a subset ("1", or "1,2"). That is
    what run_test.sh uses for the quick local pass/fail loop: the same grader
    and the same answer key, over one configuration instead of three.
    Gradescope never sets it and it is not on ENV_ALLOWLIST, so a production
    run always grades all three.
    """
    raw = os.environ.get("AUTOGRADER_CONFIGS")
    if not raw:
        return list(ALL_CONFIGS)
    chosen = []
    for piece in raw.split(","):
        piece = piece.strip()
        if not piece:
            continue
        if not piece.isdigit() or int(piece) not in ALL_CONFIGS:
            raise SystemExit(
                "AUTOGRADER_CONFIGS=%r: %r is not one of %s"
                % (raw, piece, ", ".join(str(c) for c in ALL_CONFIGS)))
        if int(piece) not in chosen:
            chosen.append(int(piece))
    if not chosen:
        raise SystemExit(
            "AUTOGRADER_CONFIGS=%r selects no configurations" % raw)
    return chosen


CONFIGS = _selected_configs()

# Which halves of the assignment this run grades. Both, normally.
#
# AUTOGRADER_PARTS restricts the run, and exists for ONE reason: the student
# hand-out built by scripts/make_student_test.sh cannot ship the assembler
# half. That half feeds the reference .s programs to the student's assembler,
# so running it requires source/reference/ on disk -- and those files are the
# very solutions students are asked to write. Setting AUTOGRADER_PARTS=assembly
# drops those tests cleanly instead of reporting five internal errors for a
# missing answer key.
#
# Gradescope never sets it, and it is not on ENV_ALLOWLIST, so a production run
# always grades both halves.
ALL_PARTS = ("assembly", "assembler")


def _selected_parts():
    raw = os.environ.get("AUTOGRADER_PARTS")
    if not raw:
        return list(ALL_PARTS)
    chosen = []
    for piece in raw.split(","):
        piece = piece.strip().lower()
        if not piece:
            continue
        if piece not in ALL_PARTS:
            raise SystemExit("AUTOGRADER_PARTS=%r: %r is not one of %s"
                             % (raw, piece, ", ".join(ALL_PARTS)))
        if piece not in chosen:
            chosen.append(piece)
    if not chosen:
        raise SystemExit("AUTOGRADER_PARTS=%r selects no parts" % raw)
    return chosen


PARTS = _selected_parts()

# THE KNOB for the assembly half. addition is worth 0: it still runs and is
# still reported as a subtest, as a diagnostic for students, but carries no
# credit. The assembly half is defined as the sum of these, so changing a
# number here re-splits the half automatically.
POINTS_ASSEMBLY = {
    "gemm": Fraction(1, 2),
    "mult": Fraction(1, 2),
    "sobel": Fraction(1, 2),
    "addition": Fraction(0),
}

# THE KNOB for the assembler half, and deliberately NOT uniform: the hidden
# all_instructions test is the one that exercises the whole instruction set, so
# it carries most of the half. addition is a 0-point diagnostic here too.
POINTS_ASSEMBLER = {
    "gemm": Fraction(1, 6),
    "mult": Fraction(1, 6),
    "sobel": Fraction(1, 6),
    "addition": Fraction(0),
    "all_instructions": Fraction(1),
}

# Derived halves. Literals here would be a second source of truth.
ASSEMBLY_TOTAL = sum(POINTS_ASSEMBLY.values(), Fraction(0))
ASSEMBLER_TOTAL = sum(POINTS_ASSEMBLER.values(), Fraction(0))
ASSIGNMENT_TOTAL = ASSEMBLY_TOTAL + ASSEMBLER_TOTAL

# Gradescope's "number" is a STRING and its ordering is lexicographic, not
# numeric. Every component here is a single digit, so
# "1.1.1" < ... < "1.4.3" < "2.1" < ... < "2.5" sorts correctly.
# If a program ever gains a TENTH config, or there is ever a tenth assembly
# program or assembler test, EVERY component at that level must be zero-padded
# ("1.1.01", "2.01"), or 10 would sort between 1 and 2.
ASSEMBLY_MAJOR = {"gemm": "1.1", "mult": "1.2", "sobel": "1.3",
                  "addition": "1.4"}
ASSEMBLER_NUMBERS = {
    "gemm": "2.1",
    "mult": "2.2",
    "sobel": "2.3",
    "addition": "2.4",
    "all_instructions": "2.5",
}

def assembly_number(program, config):
    return "%s.%d" % (ASSEMBLY_MAJOR[program], config)


def assembly_test_name(program, config):
    return "Assembly: %s (config %d/%d)" % (program, config,
                                            len(CONFIGS))


def assembly_points(program):
    """Each program's configs sum EXACTLY to its program total.

    An even split of a program total is generally not representable in binary
    floating point, so it stays a Fraction all the way to the final rounding
    and the configs add back to POINTS_ASSEMBLY[program] exactly.
    """
    return POINTS_ASSEMBLY[program] / len(CONFIGS)

# Canonical ordering + max scores, so a results.json can always be built for
# every test even if the run dies before that test was reached.
TEST_SPECS = (
    ([(assembly_number(prog, cfg), assembly_test_name(prog, cfg),
       assembly_points(prog))
      for prog in ASSEMBLY_TESTS for cfg in CONFIGS]
     if "assembly" in PARTS else [])
    + ([(ASSEMBLER_NUMBERS[n], "Assembler: %s" % n, POINTS_ASSEMBLER[n])
        for n in ASSEMBLER_TESTS]
       if "assembler" in PARTS else [])
)

# --------------------------------------------------------------------------
# Run limits
# --------------------------------------------------------------------------

RARS_TIMEOUT_SEC = 60
RARS_KILL_GRACE_SEC = 5
ASSEMBLER_TIMEOUT_SEC = 60

# The deterministic work cap is per (program, configuration) and is DATA, not
# a constant here: source/gen_expected.py measures what the reference solution
# actually costs under each configuration and writes the budget to
# source/max_cycles/<program>/cfg<N>.txt. RARS takes it as a bare integer
# argument and stops the program at that many executed instructions.
#
# It is preferred over the wall-clock timeout above because it does not depend
# on how loaded the grading machine is -- the same submission gets the same
# verdict every time. It is what fails a program that loops forever, or one
# that multiplies by repeated addition instead of shift-and-add (the two cost
# one iteration per UNIT versus per BIT of the operand, so on the large
# operands configuration 3 uses they differ by orders of magnitude).
#
# Regenerating the answer key regenerates the budgets, so the two cannot drift
# apart.

# Whole-run wall-clock cap. Worst case without it is 12 RARS runs (4 programs x
# 3 configs) plus 13 student-assembler runs (the same 12, plus all_instructions
# once) at 60s each = 1500s, which would badly outlast the Gradescope
# container. Every sub-test derives its own timeout from the budget still left,
# so a submission that hangs everywhere finishes just past this cap with its
# later tests reported as "out of time" rather than not reported at all.
GLOBAL_BUDGET_SEC = 480
MIN_SUBTEST_SEC = 5        # below this much budget, do not start a sub-test

MAX_DIFF_LINES = 25        # assembly dump diff lines shown
MAX_TEXT_MISMATCHES = 10   # assembler text-file mismatches shown
MAX_MISMATCH_CHARS = 240   # per line, per side, inside a mismatch report
# Child output is redirected to a FILE and only this much is ever read back
# into memory, from each end. Keep the total modest: whatever is read lands in
# a student-visible "output" field, and seven fields of 1 MB each would make
# results.json too big for Gradescope to render.
CAPTURE_HEAD_BYTES = 4 * 1024    # child output kept from the start ...
CAPTURE_TAIL_BYTES = 4 * 1024    # ... and from the end (8 KB total)
CHILD_FSIZE_LIMIT = 64 * 1024 * 1024   # RLIMIT_FSIZE for child processes
CHILD_AS_LIMIT = 2 * 1024 * 1024 * 1024  # RLIMIT_AS for student python
MAX_COMPARE_BYTES = 16 * 1024 * 1024   # refuse to compare bigger outputs
MAX_DUMP_LINES = 100000    # Mem[ lines kept from a RARS run
MAX_LISTING_ENTRIES = 50   # names shown when listing a directory
MAX_SUBMISSION_LISTING = 60

# RARS memory-dump ranges: start address, last word dumped (inclusive).
# Byte-identical to the programs array in run_tests.sh.
RARS_RANGES = {
    "mult": ("0x10010008", "0x10010008"),
    # addition stores its single result word at the same address as mult.
    "addition": ("0x10010008", "0x10010008"),
    "gemm": ("0x10010080", "0x100100bc"),
    "sobel": ("0x100100ac", "0x100100cc"),
}

# Students submit one .s per assembly program plus their assembler. Derived
# from ASSEMBLY_TESTS so adding a program cannot leave discovery behind.
WANTED_SUBMISSION_FILES = [p + ".s" for p in ASSEMBLY_TESTS] + ["assembler.py"]

# Pruned before directory scoring, not after: a stale copy of the four
# wanted files inside one of these must never be able to win.
JUNK_DIRS = frozenset([
    "__MACOSX", "__pycache__", ".git", "venv", ".venv", "env",
    "node_modules",
])

# The four outputs the assembler must produce:
#   (label, what the student's assembler writes, what the key calls it)
# The two differ because the generated name cannot carry a configuration --
# the student is handed <name>.s under every configuration and always writes
# <name>.hex.txt -- while the key holds one per configuration, under the
# program's own directory: expected/assembler/gemm/cfg2.hex.txt.
# All four are text, so every comparison here is the normalized-text one --
# .bin.txt is only 0/1 digits, so strip/lower is a no-op on it, but it goes
# through the same pipeline rather than being special-cased.
#
# Scoring is ALL-OR-NOTHING per test: every configuration's four files must
# match for the test to earn POINTS_ASSEMBLER[name], otherwise the test earns
# 0. Each configuration's four checks are still reported individually, as
# diagnostics, so a student can see which file is wrong and where.
ASSEMBLER_OUTPUTS = [
    ("Instruction Hex", "{n}.hex.txt", "cfg{c}.hex.txt"),
    ("Instruction Binary (bits)", "{n}.bin.txt", "cfg{c}.bin.txt"),
    ("Data Hex", "{n}_data.hex.txt", "cfg{c}_data.hex.txt"),
    ("Data Binary (bits)", "{n}_data.bin.txt", "cfg{c}_data.bin.txt"),
]

# --------------------------------------------------------------------------
# Point arithmetic, checked at import.
#
# Every split here is a Fraction, so these are EXACT equalities, not
# tolerances. Repointing the assignment is exactly the kind of edit that
# silently stops adding up; this turns that into an immediate crash.
# --------------------------------------------------------------------------

assert set(POINTS_ASSEMBLER) == set(ASSEMBLER_TESTS), \
    "POINTS_ASSEMBLER and ASSEMBLER_TESTS must cover the same tests"
assert set(POINTS_ASSEMBLY) == set(ASSEMBLY_TESTS), \
    "POINTS_ASSEMBLY and ASSEMBLY_TESTS must cover the same programs"
assert all(assembly_points(p) * len(CONFIGS) == POINTS_ASSEMBLY[p]
           for p in ASSEMBLY_TESTS), \
    "each program's configs must sum back to its program total"
assert ASSEMBLY_TOTAL == ASSEMBLER_TOTAL == Fraction(3, 2), \
    ("each half must be worth 1.5 (got %s + %s)"
     % (ASSEMBLY_TOTAL, ASSEMBLER_TOTAL))

# Tests whose .s program is NOT shipped to students. The assembler is handed
# the program as argv[1] -- it must be, that is the job -- so the file is
# readable by student code by construction and privilege separation cannot
# help. What we CAN do is refuse to echo anything student code controls back
# into a student-visible field, which is the only route from that file to the
# student. gemm/mult/sobel are shipped skeletons and stay fully verbose.
HIDDEN_TESTS = frozenset(["all_instructions"])

# Everything student-controlled that would otherwise be echoed for a hidden
# test is replaced by this.
HIDDEN_NOTICE = (
    "[output suppressed: %s is a hidden test, so the autograder does not "
    "echo anything your program produced -- that channel would reveal the "
    "test itself. Debug using the three visible programs; a bug that shows "
    "up only here is almost always an instruction your assembler has not "
    "implemented.]")


# Prepended to any sub-test whose max is 0, so a student who sees
# "Score: 0.00 / 0.00" next to four passing checks does not read it as a
# grader bug and file a regrade request.
ZERO_POINT_NOTICE = (
    "This program is a 0-point diagnostic: it is graded and reported so you "
    "can see your result, but it carries no credit.")

CONTACT_STAFF = "Please contact the course staff; this is not your fault.\n"

# --------------------------------------------------------------------------
# Run state. Kept at module level so the emergency handler can still emit
# whatever credit was already earned.
# --------------------------------------------------------------------------

RESULTS_BY_NUMBER = {}
START_TIME = time.time()


def time_left():
    return GLOBAL_BUDGET_SEC - (time.time() - START_TIME)


# --------------------------------------------------------------------------
# Bounded child-process execution
# --------------------------------------------------------------------------

def _limit_fsize():
    """Run in the child between fork and exec.

    RLIMIT_FSIZE stops a runaway child from filling the disk with the output
    we are capturing (it is killed with SIGXFSZ instead). We never read the
    whole capture file into memory regardless, so this is belt and braces.
    """
    if resource is None:
        return
    try:
        resource.setrlimit(resource.RLIMIT_FSIZE,
                           (CHILD_FSIZE_LIMIT, CHILD_FSIZE_LIMIT))
    except Exception:
        pass


def _limit_student_python():
    """Child limits for the student's assembler: file size plus address space.

    RLIMIT_AS is deliberately NOT applied to the JVM: a hard address-space cap
    makes the JVM abort at startup. RARS is bounded with -Xmx512m instead.
    """
    _limit_fsize()
    if resource is None:
        return
    try:
        resource.setrlimit(resource.RLIMIT_AS, (CHILD_AS_LIMIT, CHILD_AS_LIMIT))
    except Exception:
        pass


PR_SET_CHILD_SUBREAPER = 36


def become_subreaper():
    """Ask the kernel to re-parent orphaned descendants onto us.

    This is what makes runaway student processes killable. Killing the child's
    process group is not sufficient on its own: a student assembler that spawns
    its helper with start_new_session=True puts that helper in a BRAND NEW
    session, so it is no longer in the group we kill. Normally such an orphan
    re-parents to PID 1 and outlives the grader entirely -- long enough to
    rewrite results.json after we have exited.

    As a subreaper we become the nearest reaping ancestor instead of PID 1, so
    every orphaned descendant lands back on us, however many times it forks or
    calls setsid, and reap_stray_children() below can then find and kill it.

    Best effort: on a kernel or platform without PR_SET_CHILD_SUBREAPER the
    process-group kill still handles the ordinary case.
    """
    if not sys.platform.startswith("linux"):
        return False
    try:
        import ctypes
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        return libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) == 0
    except Exception:
        return False


def kill_process_group(proc):
    """SIGKILL the child's whole process group, then reap the child."""
    try:
        pgid = os.getpgid(proc.pid)
    except (ProcessLookupError, OSError):
        pgid = None
    # Never signal our own group: that would kill the grader.
    if pgid is not None and pgid != os.getpgrp():
        try:
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass
    try:
        proc.kill()
    except (ProcessLookupError, OSError):
        pass
    try:
        proc.wait(timeout=5)
    except Exception:
        pass


def reap_stray_children(deadline=3.0):
    """Kill every process still parented to us, and reap the zombies.

    Thanks to become_subreaper() this includes descendants that detached with
    setsid and whose immediate parent has already exited. Returns the number of
    strays that had to be killed.
    """
    killed = 0
    end = time.time() + deadline
    while True:
        # Reap anything that has already finished so it stops showing up.
        try:
            while True:
                pid, _status = os.waitpid(-1, os.WNOHANG)
                if pid == 0:
                    break
        except ChildProcessError:
            pass
        except Exception:
            pass

        strays = list_child_pids()
        if not strays or time.time() > end:
            return killed
        for pid in strays:
            for sig in (signal.SIGKILL,):
                try:
                    os.kill(pid, sig)
                    killed += 1
                except (ProcessLookupError, PermissionError, OSError):
                    pass
        time.sleep(0.05)


def list_child_pids():
    """PIDs whose parent is this process, via /proc. Empty off Linux."""
    me = os.getpid()
    found = []
    try:
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open("/proc/%s/stat" % entry, "rb") as f:
                    fields = f.read().decode("utf-8", "replace")
                # comm can contain spaces/parens, so parse after the last ')'
                tail = fields[fields.rfind(")") + 1:].split()
                if len(tail) >= 2 and int(tail[1]) == me:
                    found.append(int(entry))
            except Exception:
                continue
    except Exception:
        return []
    return found


class Privsep(object):
    """Decides the identity that untrusted student subprocesses run as.

    The rule is not "drop if convenient", it is: **running student code as root
    must be impossible**.

      euid != 0  -- native/developer mode. We cannot drop at all, so student
                    code runs as us, exactly as it always has. Supported.
      euid == 0  -- the drop is MANDATORY. We drop to the configured student
                    account if it checks out; if that account is missing or is
                    not who we expect, we still drop, just to the bare NUMERIC
                    uid/gid (setuid(1001) needs no passwd entry); and failing
                    that, to nobody. Only if EVERY candidate fails a live probe
                    do we refuse to grade at all.

    That numeric fallback is the whole point. Packaging is a Gradescope zip, so
    there is no image ENV block and no `useradd --uid 1001` whose failure breaks
    the build -- a soft-failing useradd in setup.sh is the realistic production
    fault. Under the old logic that fault silently handed every submission root.
    Here it costs us a passwd entry and nothing else.
    """

    def __init__(self):
        self.enabled = False
        self.is_root = False
        self.uid = STUDENT_UID
        self.gid = STUDENT_GID
        self.home = STUDENT_HOME
        self.home_usable = False
        self.reason = "not initialised"
        self.refusal = None

    def decide(self):
        self.is_root = (os.name == "posix" and os.geteuid() == 0)

        if os.name != "posix":
            self.reason = "not a POSIX platform"
            return False
        if not self.is_root:
            self.reason = ("grader is not root (euid %d), so student code runs "
                           "with the grader's own privileges" % os.geteuid())
            return False

        # Root from here on: we drop, or we do not grade.
        for uid, gid, home_ok, label in self._candidates():
            if probe_drop(uid, gid):
                self.uid, self.gid = uid, gid
                self.home_usable = home_ok
                self.enabled = True
                self.reason = "student subprocesses run as %s" % label
                return True

        self.refusal = (
            "Could not drop privileges to ANY unprivileged identity "
            "(tried: %s). Refusing to run untrusted submissions as root."
            % ", ".join(c[3] for c in self._candidates()))
        self.reason = self.refusal
        return False

    def _candidates(self):
        """Identities to try, best first. Each is (uid, gid, home_ok, label)."""
        out = []
        if self.configured_ids_valid():
            mismatch = self._identity_mismatch()
            if mismatch is None:
                out.append((STUDENT_UID, STUDENT_GID, True,
                            "%s (uid %d, gid %d)"
                            % (STUDENT_USER, STUDENT_UID, STUDENT_GID)))
            else:
                # P2: the name/gid did not check out. We still drop -- to the
                # numbers we were configured with -- because an unverified
                # unprivileged identity is enormously safer than root. What we
                # do NOT do is trust its passwd home directory.
                out.append((STUDENT_UID, STUDENT_GID, False,
                            "uid %d/gid %d numerically (%s)"
                            % (STUDENT_UID, STUDENT_GID, mismatch)))
        for name, uid, gid in (("nobody", 65534, 65534),):
            out.append((uid, gid, False,
                        "%s (uid %d, gid %d) as a last resort"
                        % (name, uid, gid)))
        return out

    def configured_ids_valid(self):
        return STUDENT_UID > 0 and STUDENT_GID > 0

    def _identity_mismatch(self):
        """None if uid STUDENT_UID really is the account we mean."""
        try:
            entry = pwd.getpwuid(STUDENT_UID)
        except KeyError:
            return "no passwd entry for uid %d" % STUDENT_UID
        if entry.pw_name != STUDENT_USER:
            return ("uid %d is %r, expected %r"
                    % (STUDENT_UID, entry.pw_name, STUDENT_USER))
        if entry.pw_gid != STUDENT_GID:
            return ("%s has gid %d, expected %d"
                    % (entry.pw_name, entry.pw_gid, STUDENT_GID))
        return None

    def can_grade(self):
        """Root with an unarmed sandbox must not grade at all."""
        return self.enabled or not self.is_root

    def describe(self):
        if self.enabled:
            state = "ON"
        elif self.is_root:
            # NEVER "native mode" here: a TA scanning the log would read that
            # as an expected developer run while untrusted code runs as root.
            state = "NOT ARMED, AND THIS PROCESS IS ROOT"
        else:
            state = "OFF (native mode)"
        return "privilege separation: %s -- %s" % (state, self.reason)


def probe_drop(uid, gid):
    """Fork a child, have it attempt the real drop, and report whether it took.

    Verifying up front is what lets us fail closed with a clean results.json
    instead of discovering at exec time, one subtest at a time, that the
    sandbox was never armed. The probe runs in a forked child so the grader's
    own privileges are untouched.
    """
    if os.name != "posix" or not hasattr(os, "fork"):
        return False
    try:
        pid = os.fork()
    except OSError:
        return False
    if pid == 0:
        try:
            os.setgid(gid)
            os.setgroups([])
            os.setuid(uid)
            os._exit(0 if (os.getuid() == uid and os.getgid() == gid) else 1)
        except BaseException:
            os._exit(1)
    try:
        _pid, status = os.waitpid(pid, 0)
    except OSError:
        return False
    return os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0


PRIVSEP = Privsep()


def drop_privileges():
    """Irreversibly become the student user. Runs in the child before exec.

    Order matters and is not negotiable: setgid FIRST (a process that has
    already dropped its uid can no longer change its gid), then drop every
    supplementary group -- otherwise the child keeps root's group memberships,
    including any group that can read the source tree -- and only then setuid.
    """
    os.setgid(PRIVSEP.gid)
    os.setgroups([])
    os.setuid(PRIVSEP.uid)


def child_preexec(limiter):
    """Build the preexec_fn: resource limits, then privilege drop."""
    def _preexec():
        if limiter is not None:
            limiter()
        if PRIVSEP.enabled:
            drop_privileges()
    return _preexec


# Only these are forwarded to student code. Gradescope injects submission and
# user identifiers into the grader's environment; an allowlist keeps them from
# reaching a student process that could exfiltrate them.
ENV_ALLOWLIST = ("PATH", "LANG", "TZ")


def child_env(workdir=None):
    """Environment for student code: an allowlist, plus a writable HOME/TMPDIR.

    A python3 child with an unset or unwritable HOME fails in confusing ways
    (imports, any ~/.cache write), so both HOME and TMPDIR must point somewhere
    the child's uid can actually write.
    """
    env = {}
    for key, value in os.environ.items():
        if key in ENV_ALLOWLIST or key.startswith("LC_"):
            env[key] = value
    env.setdefault("PATH", "/usr/local/bin:/usr/bin:/bin")
    env["PYTHONDONTWRITEBYTECODE"] = "1"

    # Only trust the configured home when the account actually checked out.
    # After a bare numeric drop there may be no passwd entry and no
    # /home/student at all, so HOME points at the per-run scratch dir, which
    # we have just chowned to the very uid the child will run as.
    if PRIVSEP.enabled and PRIVSEP.home_usable:
        env["HOME"] = PRIVSEP.home
    elif workdir:
        env["HOME"] = workdir
    if workdir:
        env["TMPDIR"] = workdir
    return env


def hand_to_student(path):
    """Give the student uid ownership of a scratch tree before spawning into it.

    tempfile.mkdtemp() is mode 0700 owned by root, so a uid-1001 child cannot
    even chdir into it, let alone write its output files there. Every file we
    stage inside (the .s, the copied assembler.py) has to change hands too.

    Returns a list of human-readable failures; empty means the whole tree
    changed hands.
    """
    if not PRIVSEP.enabled:
        return []

    uid, gid = PRIVSEP.uid, PRIVSEP.gid
    failures = []

    def attempt(target):
        try:
            os.chown(target, uid, gid)
        except OSError as exc:
            failures.append("%s (%s)" % (target, exc.strerror or exc))

    # Each chown is guarded individually and the walk is NOT wrapped in one
    # outer try: a single failure must not abort the descent and leave the
    # rest of the tree root-owned. Failures are returned, never swallowed --
    # an unwritable scratch dir makes every output file "missing", which
    # without this would be reported to the student as their own bug.
    attempt(path)
    try:
        for root, dirnames, filenames in os.walk(path):
            for name in dirnames + filenames:
                target = os.path.join(root, name)
                if not os.path.islink(target):
                    attempt(target)
    except OSError as exc:
        failures.append("walking %s (%s)" % (path, exc.strerror or exc))
    return failures


def keep_outputs(label, files=(), dump=None):
    """Copy what a test PRODUCED into KEEP_OUTPUTS_DIR under a stable name.

    Called just before the scratch tree is deleted. `files` are (destination
    path relative to the label, source path) pairs; `dump` is RARS's memory
    dump, which only ever exists on stdout and is written into the copy, never
    back into the scratch tree a child ran in. Nothing else is copied, so the
    staged inputs -- the student's own files, the spliced .s -- stay behind.

    Purely a local debugging aid, so every failure is swallowed: a copy that
    cannot be made must not change a score. Only named regular files are
    copied, so a symlink the student's code planted is skipped rather than
    followed out of the scratch tree.
    """
    if not KEEP_OUTPUTS_DIR:
        return
    try:
        dest_dir = os.path.join(KEEP_OUTPUTS_DIR, label)
        if dump is not None:
            os.makedirs(dest_dir, exist_ok=True)
            with open(os.path.join(dest_dir, "dump.txt"), "w") as f:
                f.write("".join(line + "\n" for line in dump))
        for relpath, source in files:
            if os.path.islink(source) or not os.path.isfile(source):
                continue
            dest = os.path.join(dest_dir, relpath)
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            shutil.copyfile(source, dest)
    except Exception:
        pass


def read_capped(fileobj, head=CAPTURE_HEAD_BYTES, tail=CAPTURE_TAIL_BYTES):
    """Read at most head+tail bytes out of a capture file, from both ends."""
    try:
        fileobj.seek(0, os.SEEK_END)
        size = fileobj.tell()
        if size <= head + tail:
            fileobj.seek(0)
            raw = fileobj.read()
        else:
            fileobj.seek(0)
            first = fileobj.read(head)
            fileobj.seek(size - tail)
            last = fileobj.read(tail)
            raw = (first
                   + ("\n... [%d bytes of output omitted] ...\n"
                      % (size - head - tail)).encode()
                   + last)
        return raw.decode("utf-8", errors="replace")
    except Exception as exc:
        return "(could not read captured output: %s)" % exc


def spawn_kwargs(handle, cwd, env, limiter):
    """Common spawn options: own session, no stdin, output to a file."""
    kwargs = {
        "stdout": handle,
        "stderr": subprocess.STDOUT,
        "stdin": subprocess.DEVNULL,
        "cwd": cwd,
        "env": env,
    }
    if os.name == "posix":
        # Own process group, so the whole tree can be killed as a unit.
        kwargs["start_new_session"] = True
        if limiter is not None:
            kwargs["preexec_fn"] = limiter
    return kwargs


def capture_size(handle):
    try:
        return os.fstat(handle.fileno()).st_size
    except Exception:
        return 0


def run_captured(cmd, timeout=None, cwd=None, env=None, limiter=None):
    """Run a command with its output captured to a file, not to a pipe.

    The child is started in its own session and its entire process group is
    SIGKILLed once we are done with it -- on success, on failure and above all
    on timeout -- followed by a sweep for detached strays. Waiting on the
    direct child alone would let a backgrounded grandchild outlive the grader.

    Returns (returncode, capped_output, timed_out, output_bytes).
    `returncode` is None when the call timed out or could not be launched.
    """
    # A missing limiter would mean a child spawned with NO privilege drop.
    # Refuse by construction rather than trusting every future caller.
    assert limiter is not None, "run_captured requires a preexec limiter"

    handle = tempfile.TemporaryFile()
    try:
        try:
            proc = subprocess.Popen(
                cmd, **spawn_kwargs(handle, cwd, env, limiter))
        except Exception as exc:
            return None, "Could not launch %s: %s" % (cmd[0], exc), False, 0

        timed_out = False
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            kill_process_group(proc)
            reap_stray_children()

        returncode = None if timed_out else proc.returncode
        return returncode, read_capped(handle), timed_out, capture_size(handle)
    finally:
        handle.close()


# What RARS prints when the step limit stops a program:
#   "Program terminated when maximum step limit 1000000 reached."
# It goes to stdout, the memory dump is STILL printed afterwards, and the exit
# code is 0 -- so without this check a program stopped mid-computation reads as
# one that simply produced wrong values.
STEP_LIMIT_NOTICE = "maximum step limit"


def stream_dump_lines(cmd, timeout, cwd, env, start_marker):
    """Run RARS and stream its capture file, keeping only the Mem[ dump lines.

    Streaming means a student who prints gigabytes before the dump costs us
    disk (capped by RLIMIT_FSIZE) but never memory.
    Returns (returncode, dump_lines, saw_marker, excerpt, timed_out,
             hit_step_limit).

    hit_step_limit is read from the whole capture file rather than from the
    bounded excerpt, so a student who floods stdout cannot push the notice out
    of the window and have the run read as a normal one.
    """
    handle = tempfile.TemporaryFile()
    try:
        try:
            proc = subprocess.Popen(
                cmd, **spawn_kwargs(handle, cwd, env,
                                    child_preexec(_limit_fsize)))
        except Exception as exc:
            return None, [], False, "Could not launch RARS: %s" % exc, False, False

        timed_out = False
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            kill_process_group(proc)
            reap_stray_children()
        returncode = None if timed_out else proc.returncode

        excerpt = read_capped(handle)

        dump_lines = []
        saw_marker = False
        hit_step_limit = False
        marker = "Mem[%s]" % start_marker
        try:
            handle.seek(0)
            for raw in handle:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if line.startswith("Mem["):
                    if marker in line:
                        saw_marker = True
                    if line.strip() and len(dump_lines) < MAX_DUMP_LINES:
                        dump_lines.append(line.rstrip())
                elif STEP_LIMIT_NOTICE in line:
                    hit_step_limit = True
        except Exception:
            pass
        return (returncode, dump_lines, saw_marker, excerpt, timed_out,
                hit_step_limit)
    finally:
        handle.close()


# --------------------------------------------------------------------------
# Submission discovery
# --------------------------------------------------------------------------

def _safe_walk(root):
    """Walk `root`, pruning junk, hidden trees and anything symlinked out.

    Yields (dirpath, filename, fullpath) for regular files only. Symlinked
    files are skipped outright: a symlink named assembler.py pointing at the
    answer key must never be selected or read.
    """
    real_root = os.path.realpath(root)
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        dirnames[:] = sorted(
            d for d in dirnames
            if not d.startswith(".")
            and d not in JUNK_DIRS
            and not os.path.islink(os.path.join(dirpath, d))
        )
        for filename in sorted(filenames):
            if filename.startswith("."):
                continue
            full = os.path.join(dirpath, filename)
            if os.path.islink(full) or not os.path.isfile(full):
                continue
            try:
                real = os.path.realpath(full)
            except OSError:
                continue
            if not (real == real_root
                    or real.startswith(real_root + os.sep)):
                continue
            yield dirpath, filename, full


def discover_submission(root=None):
    """Locate the four wanted files, however the student nested them.

    Files are chosen by DIRECTORY, not one at a time. Choosing per file
    independently lets a stale copy inside an "old backup/" folder outrank the
    real submission, and lets gemm.s and assembler.py come from two unrelated
    directories. The ranking is:

      1. most of the four wanted names present  (primary)
      2. DEEPEST                                (tie-break)
      3. sorted relpath                         (deterministic)

    Count first is what defeats a stale partial copy: an "old backup/v1/venv"
    holding 2 of the 4 loses to a complete set at any depth. Depth only decides
    between equally COMPLETE directories, and there the deeper one is the
    student's real work -- a complete stale starter set at the top level with
    the real work in project_final/src/ is exactly the common messy shape.

    Only names the winning directory lacks are searched for globally, deepest
    wins there too, for consistency.

    Matching is case-insensitive so a macOS/Windows student who submits
    Gemm.s or Assembler.py is not told the file is missing.

    Returns (found, all_files, chosen_dir); `found` maps the canonical
    lowercase basename to a full path.
    """
    if root is None:
        root = SUBMISSION_DIR

    wanted = set(WANTED_SUBMISSION_FILES)
    all_files = []
    by_dir = {}      # dirpath -> {lowername: fullpath}
    by_name = {}     # lowername -> [(depth, relpath, fullpath), ...]

    if not os.path.isdir(root):
        return {}, [], None

    for dirpath, filename, full in _safe_walk(root):
        rel = os.path.relpath(full, root)
        all_files.append(rel)
        lower = filename.lower()
        if lower not in wanted:
            continue
        # First in sorted order wins within one directory (Gemm.s vs gemm.s).
        by_dir.setdefault(dirpath, {}).setdefault(lower, full)
        by_name.setdefault(lower, []).append((rel.count(os.sep), rel, full))

    found = {}
    chosen_dir = None
    if by_dir:
        def dir_rank(item):
            dirpath, names = item
            rel = os.path.relpath(dirpath, root)
            depth = 0 if rel == "." else rel.count(os.sep) + 1
            # count desc, then DEEPEST, then name
            return (-len(names), -depth, rel)

        best_dir, best_names = sorted(by_dir.items(), key=dir_rank)[0]
        found.update(best_names)
        chosen_dir = best_dir

    for name in WANTED_SUBMISSION_FILES:
        if name in found or name not in by_name:
            continue
        # Deepest wins; among equally deep, first in sorted order.
        found[name] = sorted(by_name[name], key=lambda m: (-m[0], m[1]))[0][2]

    return found, sorted(all_files), chosen_dir


def describe_submission(all_files):
    """Student-facing listing of what was actually in the upload."""
    if not all_files:
        return "No files were found in your submission.\n"
    lines = ["Files found in your submission:"]
    for rel in all_files[:MAX_SUBMISSION_LISTING]:
        lines.append("- " + rel)
    if len(all_files) > MAX_SUBMISSION_LISTING:
        lines.append("... and %d more file(s)."
                     % (len(all_files) - MAX_SUBMISSION_LISTING))
    return "\n".join(lines) + "\n"


def describe_choice(found, names, chosen_dir=None, root=None):
    """Tell the student exactly which uploaded files the grader picked."""
    if root is None:
        root = SUBMISSION_DIR
    picked = []
    if chosen_dir is not None:
        try:
            shown_dir = os.path.relpath(chosen_dir, root)
        except ValueError:
            shown_dir = chosen_dir
        picked.append("- graded from directory: %s"
                      % ("<top level of your submission>"
                         if shown_dir == "." else shown_dir))
    for name in names:
        if name in found:
            try:
                shown = os.path.relpath(found[name], root)
            except ValueError:
                shown = found[name]
            picked.append("- %s -> %s" % (name, shown))
    if not picked:
        return ""
    return "Files selected from your submission:\n" + "\n".join(picked) + "\n"


# --------------------------------------------------------------------------
# Comparison against the preloaded answer key
# --------------------------------------------------------------------------

def normalize_lines(raw_lines):
    """Ignore blank lines, surrounding spaces and hex letter case."""
    return [line.strip().lower() for line in raw_lines if line.strip()]


def load_text_lines(path):
    """Normalized lines of a text file. Raises on failure."""
    with open(path, "r", newline=None, errors="replace") as f:
        return normalize_lines(f)


def read_generated_text(path):
    """Normalized lines of a student-produced text file.

    Returns (lines, error). Oversized files are refused rather than read, so a
    student cannot make the grader eat their output.
    """
    try:
        if os.path.getsize(path) > MAX_COMPARE_BYTES:
            return None, ("The generated file is larger than %d MB, which is "
                          "far bigger than any correct output; it was not "
                          "compared." % (MAX_COMPARE_BYTES // (1024 * 1024)))
        return load_text_lines(path), None
    except Exception as exc:
        return None, "Could not read the generated file: %s" % exc


def clip(text, limit=MAX_MISMATCH_CHARS):
    """Shorten one reported line.

    Without this, ten whole lines per file per test are copied verbatim into
    results.json; an assembler emitting megabyte-long lines turns that into a
    results.json hundreds of megabytes big.
    """
    if len(text) <= limit:
        return text
    return "%s... [%d more chars]" % (text[:limit], len(text) - limit)


def text_line_mismatches(generated, expected, max_mismatches=MAX_TEXT_MISMATCHES):
    """Line-by-line mismatches under normalize_lines() semantics."""
    mismatches = []
    for i in range(max(len(generated), len(expected))):
        gen_line = generated[i] if i < len(generated) else "<missing line>"
        exp_line = expected[i] if i < len(expected) else "<extra generated line>"
        if gen_line != exp_line:
            mismatches.append({"line": i + 1,
                               "expected": clip(exp_line),
                               "generated": clip(gen_line)})
            if len(mismatches) >= max_mismatches:
                break
    return mismatches


def list_directory_capped(directory):
    """Names in a directory, capped. An assembler that creates a million files
    in gen/ must not be able to inflate results.json without limit."""
    names = []
    total = 0
    try:
        with os.scandir(directory) as it:
            for entry in it:
                total += 1
                if len(names) < MAX_LISTING_ENTRIES:
                    names.append(entry.name)
                elif total > MAX_LISTING_ENTRIES * 200:
                    break
    except Exception as exc:
        return None, "(could not list %s: %s)" % (directory, exc)
    names.sort()
    suffix = ""
    if total > len(names):
        suffix = "... and %d more entr%s.\n" % (
            total - len(names), "y" if total - len(names) == 1 else "ies")
    return names, suffix


def missing_output_message(filename, search_dirs, hidden=False):
    """Explain a missing assembler output, listing what was produced instead."""
    message = (
        "Missing output file: %s\n"
        "Your assembler did not generate this required file.\n" % filename
    )
    if hidden:
        # Filenames are chosen by the student, so listing them is another
        # channel out of the hidden test.
        return message + (
            "\nCheck that the filename matches exactly, including "
            "underscores, extensions, and .hex.txt / .bin.txt endings.\n")
    listed = False
    for directory in search_dirs:
        if not os.path.isdir(directory):
            continue
        names, suffix = list_directory_capped(directory)
        if names is None:
            message += "\n" + suffix + "\n"
            continue
        if names:
            message += "\nFiles found in output folder (%s):\n" % (
                os.path.basename(directory.rstrip(os.sep)) or directory)
            for name in names:
                message += "- %s\n" % name
            message += suffix
            listed = True
    if not listed:
        message += "\nNo output files were found at all.\n"
    message += (
        "\nCheck that the filename matches exactly, including underscores, "
        "extensions, and .hex.txt endings.\n"
    )
    return message


def format_check_failure(check):
    """Student-facing message for one failed file check."""
    kind = check["error_type"]
    details = check["details"]
    if kind == "missing_file":
        return details["message"]
    if kind == "unreadable":
        return details["message"] + "\n"
    if kind == "content_mismatch":
        lines = ["The file was generated, but its contents are incorrect."]
        if details.get("hidden_mismatch"):
            first = details.get("first_lines") or []
            lines.append("%d line(s) differ%s."
                         % (details.get("count", 0),
                            (", first at line " +
                             ", ".join(str(n) for n in first)) if first else ""))
            lines.append("Line contents are not shown for the hidden test.")
            return "\n".join(lines) + "\n"
        mismatches = details.get("line_mismatches")
        if mismatches:
            lines.append("")
            lines.append("Mismatched lines:")
            for m in mismatches:
                lines.append("Line %s:" % m["line"])
                lines.append("  Expected : %s" % m["expected"])
                lines.append("  Generated: %s" % m["generated"])
        return "\n".join(lines) + "\n"
    return "Unknown error.\n"


# --------------------------------------------------------------------------
# Preload: read the entire answer key into memory before any student code runs
# --------------------------------------------------------------------------

class AnswerKey(object):
    """The whole answer key, held in memory.

    Loaded before a single line of student code executes, so student code that
    rewrites the expected files on disk (RARS 1.6 implements the file-write
    ecalls, and assembler.py is ordinary Python) changes nothing about how it
    is graded.
    """

    def __init__(self):
        self.assembly = {}        # (program, config) -> [dump line, ...]
        self.max_steps = {}       # (program, config) -> instruction budget
        self.configs = {}         # (program, config) -> .data block text
        # name -> {config -> {filename: [normalized lines]}}
        self.assembler = {}
        # (name, config) -> the bytes handed to the student's assembler, i.e.
        # reference/<name>.s with that configuration's .data already spliced in.
        self.assembler_sources = {}
        self.errors = {}          # ("assembly"|"assembler", name) -> message
        self.global_error = None

    def fail(self, kind, name, message):
        self.errors[(kind, name)] = message


class InternalGradingError(Exception):
    """An autograder-side fault, never the student's doing.

    guarded() renders this as a plain INTERNAL error without a traceback, so a
    broken image reads as a broken image rather than as a bad submission.
    """


def chown_failure_message(failures):
    return ("Could not hand the scratch directory to the sandbox user "
            "(uid %d, gid %d); student code would not have been able to write "
            "its output files, so this test could not be run.\n\n"
            "%d failure(s):\n%s"
            % (PRIVSEP.uid, PRIVSEP.gid, len(failures),
               "\n".join("  - " + f for f in failures[:10])))


def internal_error_text(message):
    return "INTERNAL AUTOGRADER ERROR\n\n" + message + "\n" + CONTACT_STAFF


def budget_exhausted_text():
    """Out of time is a fact about the submission, not an autograder fault."""
    return (
        "OUT OF TIME\n\n"
        "The autograder's overall time limit of %d seconds was used up before "
        "this test could run, so it scored 0.\n"
        "This normally means an earlier test ran very slowly or looped "
        "forever. Fix the timeouts reported above and this test will run.\n\n"
        % GLOBAL_BUDGET_SEC)


def find_git_worktree(start):
    """Return the .git path if `start` sits inside a git working tree."""
    current = os.path.realpath(start)
    while True:
        candidate = os.path.join(current, ".git")
        if os.path.exists(candidate):
            return candidate
        parent = os.path.dirname(current)
        if parent == current:
            return None
        current = parent


def guarded_removable(target, expected_basename):
    """Return (ok, reason) for deleting `target`.

    Three independent guards, because this is the only destructive operation
    in the grader and it must refuse rather than guess.
    """
    if not os.path.exists(target):
        return False, "nothing to remove"

    real_target = os.path.realpath(target)
    real_root = os.path.realpath(AUTOGRADER_ROOT)

    # Guard 1: never touch anything outside AUTOGRADER_ROOT.
    try:
        inside = os.path.commonpath([real_root, real_target]) == real_root
    except ValueError:          # different drives / unrelated roots
        inside = False
    if not inside or real_target == real_root:
        return False, "%s is not safely inside %s" % (real_target, real_root)

    # Guard 2: never touch a developer checkout.
    git_dir = find_git_worktree(os.path.dirname(real_target))
    if git_dir is not None:
        return False, ("this looks like a developer checkout (found %s)"
                       % git_dir)

    # Guard 3: it really is the directory we think it is.
    if os.path.basename(real_target) != expected_basename:
        return False, "%s is not named %r" % (real_target, expected_basename)

    return True, real_target


def remove_on_disk_answer_key():
    """Delete every on-disk copy of the answer key once it is safely in memory.

    Defense in depth only: the in-memory preload is what actually guarantees
    correctness. Removing the tree additionally means student code cannot copy
    the key into its own output directory to score full marks without
    assembling anything.

    This is the one destructive operation in the grader, so it is fenced by
    two independent guards and refuses rather than guesses.

    Returns a human-readable note about what it did.
    """
    # source/outputs holds RARS dumps byte-identical to expected/assembly, so
    # it is a second, equally exploitable copy of the assembly answer key.
    # source/reference holds the reference assembler and the reference .s
    # programs -- including the hidden all_instructions.s -- and every byte of
    # it that grading needs was read into key.assembler_sources above.
    targets = [
        (os.path.join(SOURCE_DIR, "expected"), "expected"),
        (os.path.join(SOURCE_DIR, "outputs"), "outputs"),
        (REFERENCE_DIR, "reference"),
    ]

    notes = []
    for target, basename in targets:
        ok, detail = guarded_removable(target, basename)
        if not ok:
            if detail == "nothing to remove":
                notes.append("%s: nothing to remove" % basename)
            else:
                notes.append("%s: NOT removed, %s" % (basename, detail))
            continue
        try:
            shutil.rmtree(detail)
            notes.append("%s: removed %s" % (basename, detail))
        except Exception as exc:
            notes.append("%s: could not remove %s: %s" % (basename, detail, exc))

    return ("on-disk answer key (in memory already, so grading is unaffected "
            "either way) -- " + "; ".join(notes) + ".")


def find_reference_program(name):
    """Path to the one file named <name>.s anywhere under source/reference/.

    Two files with that name is the one case that cannot be resolved by
    looking, so it is an error rather than a guess -- grading the wrong copy of
    a reference program would be silent and wrong.
    """
    filename = name + ".s"
    matches = []
    for dirpath, dirnames, filenames in os.walk(REFERENCE_DIR):
        dirnames[:] = [d for d in dirnames if d not in ("__pycache__", "gen")]
        if filename in filenames:
            matches.append(os.path.join(dirpath, filename))
    if not matches:
        raise ValueError("no file named %s anywhere under %s"
                         % (filename, REFERENCE_DIR))
    if len(matches) > 1:
        raise ValueError("%d files named %s under %s: %s"
                         % (len(matches), filename, REFERENCE_DIR,
                            ", ".join(sorted(matches))))
    return matches[0]


def load_step_budget(path):
    """The instruction budget in one max_cycles file.

    The file is the number on its own line; `#` lines above it record what the
    reference cost and how the budget was derived, for whoever reads it later.
    """
    with open(path, "r", newline=None, errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            budget = int(line)
            if budget <= 0:
                raise ValueError("budget must be positive, got %d" % budget)
            return budget
    raise ValueError("no budget line in the file")


def load_assembler_key(name, reference, configs):
    """The expected outputs and spliced programs for one assembler test.

    Returns ({config: {filename: [normalized lines]}},
             {config: program bytes}).
    Raises ValueError with a staff-facing message if the image is incomplete.

    The splicing happens HERE, at preload time, so a reference program that
    cannot be spliced is an internal error reported before any student code
    starts. SPLICE is defined further down the file and resolved when this is
    called, which is long after import.
    """
    expected = {}
    sources = {}
    for config in CONFIGS:
        try:
            spliced = SPLICE(reference.decode("utf-8", "replace"),
                             configs[(name, config)])
        except Exception as exc:
            raise ValueError(
                "Could not splice configuration %d into %s.s: %s"
                % (config, name, exc))
        sources[config] = spliced.encode("utf-8")

        files = {}
        # Keyed by the GENERATED name, which is what the check looks up.
        for _label, gen_pattern, key_pattern in ASSEMBLER_OUTPUTS:
            path = expected_assembler_path(name, key_pattern.format(c=config))
            try:
                files[gen_pattern.format(n=name)] = load_text_lines(path)
            except Exception as exc:
                raise ValueError("Could not load the expected output %s: %s"
                                 % (path, exc))
        expected[config] = files
    return expected, sources


def preload_answer_key():
    """Read every expected file and every test program.

    Any resource that is missing or unreadable is recorded as an INTERNAL
    error against the tests that need it, so a broken image is never reported
    to the student as a mistake in their own code.
    """
    key = AnswerKey()

    if not os.path.isfile(RARS_JAR):
        key.global_error = "RARS jar is missing from the image: %s" % RARS_JAR
    else:
        rars_ok = None
        if shutil.which("java") is None:
            rars_ok = "java is not installed in the grading image."
        if rars_ok:
            key.global_error = rars_ok

    # ASSEMBLER_TESTS is the superset: every program that has a configuration
    # directory, including the assembler-only all_instructions. With the
    # assembler half switched off, all_instructions is not graded and its
    # configuration directory need not be present at all.
    config_programs = (ASSEMBLER_TESTS if "assembler" in PARTS
                       else ASSEMBLY_TESTS)
    for program in config_programs:
        for config in CONFIGS:
            slot = (program, config)
            # The .data block is loaded even when RARS is unusable: the
            # assembler half splices the same block and needs no java.
            block = os.path.join(CONFIGS_DIR, program, "%d.data" % config)
            try:
                with open(block, "r", newline=None, errors="replace") as f:
                    key.configs[slot] = f.read()
            except Exception as exc:
                message = ("Could not load the .data configuration %s: %s"
                           % (block, exc))
                if program in ASSEMBLY_TESTS:
                    key.fail("assembly", slot, message)
                key.fail("assembler", program, message)
                continue

            # An assembler-only program is never run, so it has no expected
            # memory dump and needs no java.
            if program not in ASSEMBLY_TESTS:
                continue

            if key.global_error:
                key.fail("assembly", slot, key.global_error)
                continue

            path = expected_assembly_path(program, config)
            try:
                with open(path, "r", newline=None, errors="replace") as f:
                    lines = [line.rstrip() for line in f if line.strip()]
                if not lines:
                    raise ValueError("expected dump file is empty")
                key.assembly[slot] = lines
            except Exception as exc:
                key.fail("assembly", slot,
                         "Could not load the expected memory dump %s: %s"
                         % (path, exc))
                continue

            path = max_cycles_path(program, config)
            try:
                key.max_steps[slot] = load_step_budget(path)
            except Exception as exc:
                key.fail("assembly", slot,
                         "Could not load the instruction budget %s: %s"
                         % (path, exc))

    for name in (ASSEMBLER_TESTS if "assembler" in PARTS else []):
        # A .data block that failed to load above already failed this test.
        if ("assembler", name) in key.errors:
            continue
        try:
            program = find_reference_program(name)
            with open(program, "rb") as f:
                reference = f.read()
        except Exception as exc:
            key.fail("assembler", name,
                     "Could not load the reference program %s.s: %s"
                     % (name, exc))
            continue

        try:
            expected, sources = load_assembler_key(name, reference, key.configs)
        except ValueError as exc:
            key.fail("assembler", name, str(exc))
            continue
        key.assembler[name] = expected
        for config, spliced in sources.items():
            key.assembler_sources[(name, config)] = spliced

    return key


# --------------------------------------------------------------------------
# Part 1: assembly tests
# --------------------------------------------------------------------------

def load_splice():
    """Return configs/splice.py's splice(), or an identical local copy.

    The expected dumps were generated with configs/splice.py, so the rule must
    match it exactly. Importing the shipped module keeps a single source of
    truth; the inline fallback exists only so a missing configs/splice.py
    degrades to still-correct grading instead of zeroing all 1.5 assembly
    points. The two implementations are verified byte-identical.
    """
    path = os.path.join(CONFIGS_DIR, "splice.py")
    saved = sys.dont_write_bytecode
    try:
        # Importing normally drops a __pycache__ into the source tree, which
        # in the image is a root-owned 0700 directory we have no business
        # mutating (and which dirties the repo on a native run).
        sys.dont_write_bytecode = True
        import importlib.util
        spec = importlib.util.spec_from_file_location("_ag_splice", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.splice
    except Exception:
        return _splice_fallback
    finally:
        sys.dont_write_bytecode = saved


def _find_directive(lines, directive, start=0):
    for i in range(start, len(lines)):
        if lines[i].strip().startswith(directive):
            return i
    return -1


def _splice_fallback(src_text, data_block_text):
    """Byte-identical reimplementation of configs/splice.py::splice."""
    lines = src_text.splitlines(keepends=True)
    d = _find_directive(lines, ".data")
    if d < 0:
        raise ValueError("no line beginning with .data")
    t = _find_directive(lines, ".text", d)
    if t < 0:
        if _find_directive(lines, ".text") >= 0:
            raise ValueError(".text appears before .data")
        raise ValueError("no line beginning with .text")
    block = data_block_text
    if block and not block.endswith("\n"):
        block += "\n"
    return "".join(lines[:d]) + block + "".join(lines[t:])


SPLICE = load_splice()


def diagnose_splice(src_text, asm_name):
    """Explain, in the student's terms, why their file cannot be spliced.

    These are all faults in a STUDENT file, so every one of them is a graded
    failure with a specific message -- never an internal error and never a
    traceback.
    """
    lines = src_text.splitlines(keepends=True)
    stripped = [line.strip() for line in lines]

    data_lines = [i for i, t in enumerate(stripped) if t.startswith(".data")]
    text_lines = [i for i, t in enumerate(stripped) if t.startswith(".text")]
    section_data = [i for i, t in enumerate(stripped)
                    if t.startswith(".section") and ".data" in t]
    section_text = [i for i, t in enumerate(stripped)
                    if t.startswith(".section") and ".text" in t]

    how = ("Your .data section is replaced by the configuration under test, so "
           "it must be findable: the grader replaces everything from the first "
           "line starting with `.data` up to the first following line starting "
           "with `.text`.\n")

    if not data_lines and section_data:
        return ("%s uses the `.section .data` spelling on line %d, which this "
                "grader does not accept.\nWrite a plain `.data` directive on its "
                "own line instead (and `.text`, not `.section .text`).\n\n%s"
                % (asm_name, section_data[0] + 1, how))
    if not data_lines:
        return ("%s has no `.data` directive.\nEvery program needs a `.data` "
                "section holding the labels the tests read.\n\n%s"
                % (asm_name, how))

    if not text_lines and section_text:
        return ("%s uses the `.section .text` spelling on line %d, which this "
                "grader does not accept.\nWrite a plain `.text` directive on "
                "its own line instead.\n\n%s"
                % (asm_name, section_text[0] + 1, how))
    if not text_lines:
        return ("%s has no `.text` directive.\nThe grader needs it to know "
                "where your `.data` section ends.\n\n%s" % (asm_name, how))

    if all(t < data_lines[0] for t in text_lines):
        return ("%s has its `.text` section (line %d) BEFORE its `.data` "
                "section (line %d).\nPut `.data` first, then `.text`.\n\n%s"
                % (asm_name, text_lines[0] + 1, data_lines[0] + 1, how))

    if len(data_lines) > 1:
        return ("%s has %d `.data` directives (lines %s).\nOnly the first can "
                "be replaced by the configuration under test, so any later "
                "`.data` would keep values the test meant to override. Use a "
                "single `.data` section.\n\n%s"
                % (asm_name, len(data_lines),
                   ", ".join(str(i + 1) for i in data_lines), how))

    return None


def splice_student_source(src_text, block, asm_name):
    """(spliced_text, None) or (None, student-facing error message)."""
    problem = diagnose_splice(src_text, asm_name)
    if problem:
        return None, problem
    try:
        return SPLICE(src_text, block), None
    except ValueError as exc:
        return None, ("%s could not be prepared for grading: %s\n"
                      % (asm_name, exc))


def grade_assembly(program, config, key, found, all_files, chosen_dir):
    """Grade one program under one .data configuration.

    Returns (passed, output). Scoring is all-or-nothing per sub-test, so the
    verdict is the whole result and guarded() turns it into points -- which is
    what lets a 0-point diagnostic test still report PASS or FAIL.

    The student supplies the .text; the .data comes from the configuration, so
    a .text that hardcodes the shipped skeleton's answer fails configs 2 and 3.
    """
    slot = (program, config)
    if ("assembly", slot) in key.errors:
        return False, internal_error_text(key.errors[("assembly", slot)])

    name = program
    asm_name = program + ".s"
    # Computed before every early return: the student who most needs to see
    # which file was graded is exactly the one whose run timed out, produced no
    # dump, or whose file was not found at all.
    picked = describe_choice(found, [asm_name], chosen_dir)

    student_asm = found.get(asm_name)
    if not student_asm:
        return False, (
            "Could not find %s in your submission.\n"
            "Upload one assembly program per test (%s), plus assembler.py "
            "(the folder they sit in does not matter).\n\n%s"
            % (asm_name, ", ".join(p + ".s" for p in ASSEMBLY_TESTS),
               describe_submission(all_files))
        )

    budget = time_left()
    if budget < MIN_SUBTEST_SEC:
        return False, budget_exhausted_text() + picked

    # Splice the configuration's .data into the student's file before it ever
    # reaches RARS. Faults here are faults in the student's file.
    try:
        with open(student_asm, "r", newline=None, errors="replace") as f:
            student_text = f.read()
    except Exception as exc:
        return False, ("Could not read %s: %s\n\n%s"
                             % (asm_name, exc, picked))

    spliced, problem = splice_student_source(
        student_text, key.configs[slot], asm_name)
    if problem:
        return False, (problem + "\n" + picked)

    start, end = RARS_RANGES[name]
    max_steps = key.max_steps[slot]
    workdir = tempfile.mkdtemp(prefix="rars_%s_cfg%d_" % (name, config))
    got = []            # bound before the try so the finally can always keep it
    try:
        # Run from a scratch directory so relative file writes by the student's
        # program land here and not anywhere near the source tree.
        local = os.path.join(workdir, asm_name)
        with open(local, "w") as f:
            f.write(spliced)
        failures = hand_to_student(workdir)
        if failures:
            raise InternalGradingError(chown_failure_message(failures))

        limit = max(MIN_SUBTEST_SEC, min(RARS_TIMEOUT_SEC, int(budget)))
        # The bare integer is RARS's maximum-steps option. The wall-clock
        # `timeout` stays as the backstop for a run that hangs OUTSIDE the
        # simulated program (a jar that will not start, a blocked read).
        cmd = [
            "timeout", "--kill-after=%ds" % RARS_KILL_GRACE_SEC, "%ds" % limit,
            "java", "-Xmx512m", "-Djava.awt.headless=true", "-jar", RARS_JAR,
            "nc", "ae1", "se1", str(max_steps),
            asm_name, "%s-%s" % (start, end),
        ]
        (returncode, got, saw_marker, excerpt, timed_out,
         hit_step_limit) = stream_dump_lines(
            cmd, timeout=limit + RARS_KILL_GRACE_SEC + 15,
            cwd=workdir, env=child_env(workdir), start_marker=start)
    finally:
        keep_outputs("rars_%s_cfg%d" % (name, config), dump=got)
        shutil.rmtree(workdir, ignore_errors=True)

    # `timeout` reports 124, or 137 when the KILL grace elapsed.
    if timed_out or returncode in (124, 137):
        return False, (
            "TIMEOUT: %s under config %d/%d did not finish within %d "
            "seconds and was killed.\n"
            "Check for an infinite loop or a missing exit syscall.\n\n"
            "RARS output:\n%s\n\n%s"
            % (asm_name, config, len(CONFIGS), limit, excerpt, picked)
        )

    # Checked before the dump is compared: RARS prints the memory dump even
    # when the step limit stopped the program, so this would otherwise surface
    # as "your values are wrong" -- which sends a student looking for an
    # arithmetic bug when the real problem is that the program never finished.
    if hit_step_limit:
        return False, (
            "INSTRUCTION LIMIT: %s under config %d/%d executed %s "
            "instructions without reaching its exit ecall, and was stopped.\n"
            "\n"
            "%s is this program's budget for this configuration: what the "
            "reference solution itself needs, plus a margin. Going over it is "
            "almost always one of:\n"
            "  * a loop that never exits -- check the condition, and check "
            "that the counter it tests is actually updated;\n"
            "  * multiplication by repeated addition instead of the "
            "shift-and-add algorithm. Repeated addition takes one iteration "
            "per UNIT of the operand; shift-and-add takes one per BIT, so 32 "
            "of them. On the large operands these configurations use, that is "
            "the difference between thousands of instructions and millions; "
            "or\n"
            "  * an algorithm that is simply doing much more work than it "
            "needs to -- an extra pass over the data, or a recomputation "
            "inside a loop that belongs outside it.\n"
            "\n"
            "Your program may well be correct on small inputs and still fail "
            "here.\n\nRARS output:\n%s\n\n%s"
            % (asm_name, config, len(CONFIGS), format(max_steps, ","),
               format(max_steps, ","), excerpt, picked)
        )

    # RARS exits 0 even when assembly fails, so the dump line is the only
    # reliable signal that the program actually ran.
    if not saw_marker:
        return False, (
            "%s under config %d/%d did not produce a memory dump "
            "(no Mem[%s] line).\n"
            "This usually means the program failed to assemble or crashed "
            "before it finished.\n\nRARS output:\n%s\n\n%s"
            % (asm_name, config, len(CONFIGS), start, excerpt, picked)
        )

    expected = key.assembly[slot]

    if got == expected:
        return True, (
            "PASS: %s produced the correct result memory under config %d/%d "
            "(%s-%s, %d word(s)).\n\n%s"
            % (asm_name, config, len(CONFIGS), start, end,
               len(expected), picked)
        )

    lines = [
        "FAIL: under config %d/%d the result memory of %s does not match the "
        "expected values." % (config, len(CONFIGS), asm_name),
        "This configuration supplies its own .data; your .text must compute "
        "the result from whatever values are there.",
        "Dump range: %s-%s" % (start, end),
        "",
        "Differences:",
    ]
    shown = 0
    total = 0
    for i in range(max(len(got), len(expected))):
        g = got[i] if i < len(got) else "<missing line>"
        e = expected[i] if i < len(expected) else "<extra line>"
        if g != e:
            total += 1
            if shown < MAX_DIFF_LINES:
                shown += 1
                lines.append("Line %d:" % (i + 1))
                lines.append("  Expected : %s" % e)
                lines.append("  Got      : %s" % g)
    if total > shown:
        lines.append("... %d more differing line(s) not shown." % (total - shown))
    lines.append("Total differing lines: %d" % total)
    lines.append("")
    lines.append(picked)
    return False, "\n".join(lines)


# --------------------------------------------------------------------------
# Part 2: assembler tests
# --------------------------------------------------------------------------

def locate_output(tmpdir, filename):
    """The reference assembler writes into gen/ next to the .s file; some
    students write into the .s file's directory instead. Accept both."""
    for directory in (os.path.join(tmpdir, "gen"), tmpdir):
        candidate = os.path.join(directory, filename)
        if os.path.isfile(candidate) and not os.path.islink(candidate):
            return candidate
    return None


def rundir_path(workdir, config):
    """One run directory per configuration, so one configuration's gen/ outputs
    can never be picked up as the next one's."""
    return os.path.join(workdir, "run_cfg%d" % config)


def generated_assembler_outputs(workdir, name, configs):
    """(relative destination, source path) for each output file the student's
    assembler produced, for keep_outputs. A file it did not write is skipped."""
    for config in configs:
        rundir = rundir_path(workdir, config)
        for _label, gen_pattern, _key in ASSEMBLER_OUTPUTS:
            gen_filename = gen_pattern.format(n=name)
            source = locate_output(rundir, gen_filename)
            if source:
                yield os.path.join(os.path.basename(rundir),
                                   gen_filename), source


def stage_student_assembler(assembler_py, workdir):
    """Copy the student's assembler (and its directory) into the scratch tree.

    Running the copy keeps argv[0] from leaking the submission path, and with
    it the layout of the autograder tree. The whole directory is copied so a
    multi-file assembler's local imports still resolve.
    """
    staged_dir = os.path.join(workdir, "student")
    source_dir = os.path.dirname(os.path.abspath(assembler_py))
    basename = os.path.basename(assembler_py)
    try:
        shutil.copytree(source_dir, staged_dir, symlinks=False,
                        ignore=shutil.ignore_patterns("__pycache__", ".*"))
        staged = os.path.join(staged_dir, basename)
        if os.path.isfile(staged):
            return staged
    except Exception:
        pass
    # Fall back to copying just the one file.
    os.makedirs(staged_dir, exist_ok=True)
    staged = os.path.join(staged_dir, "assembler.py")
    shutil.copyfile(assembler_py, staged)
    return staged


def student_visible_output(name, output):
    """What of a child's own output may be shown for this test."""
    if name in HIDDEN_TESTS:
        return HIDDEN_NOTICE % (name + ".s")
    return output


def run_student_assembler(key, name, config, staged, workdir, budget):
    """Run the student's assembler on one configuration of a reference program.

    Returns (rundir, failure). `failure` None means the run finished and
    whatever it left in rundir can be compared.
    """
    rundir = rundir_path(workdir, config)
    os.makedirs(rundir, exist_ok=True)

    dest = os.path.join(rundir, name + ".s")
    with open(dest, "wb") as f:
        f.write(key.assembler_sources[(name, config)])

    # Everything is staged; only now hand the whole scratch tree to the
    # student uid, so the child can enter it and write its gen/ outputs.
    failures = hand_to_student(workdir)
    if failures:
        raise InternalGradingError(chown_failure_message(failures))

    limit = max(MIN_SUBTEST_SEC, min(ASSEMBLER_TIMEOUT_SEC, int(budget)))
    cmd = [sys.executable or "python3", staged, dest]
    returncode, output, timed_out, output_bytes = run_captured(
        cmd, timeout=limit, cwd=rundir, env=child_env(rundir),
        limiter=child_preexec(_limit_student_python))

    output = student_visible_output(name, output)

    if timed_out:
        return rundir, (
            "TIMEOUT: your assembler did not finish within %d seconds on %s.s "
            "and was killed.\nCheck for an infinite loop.\n\nOutput so far:\n%s\n"
            % (limit, name, output)
        )
    if returncode is None:
        return rundir, "Could not run your assembler.\n\n%s\n" % output
    if returncode != 0:
        # SIGXFSZ (or Python's own 120 when it cannot flush) after the capture
        # hit the cap means the run drowned in its own output, not that the
        # code threw. Saying "crashed" there sends students hunting a bug that
        # is not the problem.
        if output_bytes >= CHILD_FSIZE_LIMIT or returncode == -signal.SIGXFSZ:
            return rundir, (
                "Your assembler produced far too much output on %s.s and was "
                "stopped after %d MB.\n"
                "Remove debugging prints from your assembler; only the output "
                "files are graded.\n\nFirst and last of what it printed:\n%s\n"
                % (name, CHILD_FSIZE_LIMIT // (1024 * 1024), output)
            )
        return rundir, (
            "Your assembler crashed on %s.s (exit code %d).\n\n"
            "It was invoked as:  python3 assembler.py %s.s\n\n"
            "Output:\n%s\n" % (name, returncode, name, output)
        )
    return rundir, None


def check_assembler_outputs(key, name, config, rundir):
    """Run the four file checks for one configuration, against memory."""
    expected = key.assembler[name][config]
    hidden = name in HIDDEN_TESTS
    checks = []
    search_dirs = [os.path.join(rundir, "gen"), rundir]

    for label, gen_pattern, _key in ASSEMBLER_OUTPUTS:
        filename = gen_pattern.format(n=name)
        check = {"name": label, "passed": False,
                 "error_type": None, "details": {}}

        gen_path = locate_output(rundir, filename)
        if gen_path is None:
            check["error_type"] = "missing_file"
            check["details"] = {
                "message": missing_output_message(filename, search_dirs,
                                                  hidden=hidden)
            }
            checks.append(check)
            continue

        generated, problem = read_generated_text(gen_path)
        if problem:
            check["error_type"] = "unreadable"
            check["details"] = {"message": problem}
        elif generated == expected[filename]:
            check["passed"] = True
        else:
            check["error_type"] = "content_mismatch"
            mismatches = text_line_mismatches(generated, expected[filename])
            if hidden:
                # The "Generated" side is student-controlled text, so echoing
                # it is an exfiltration channel; the "Expected" side is the
                # answer key. For a hidden test show neither -- only where the
                # first differences are, which is what actually helps.
                check["details"] = {
                    "hidden_mismatch": True,
                    "first_lines": [m["line"] for m in mismatches[:5]],
                    "count": len(mismatches),
                }
            else:
                check["details"] = {"line_mismatches": mismatches}
        checks.append(check)
    return checks


def config_passed(result):
    """A configuration passed only if it ran AND all four of its files matched."""
    return (result["failure"] is None
            and all(c["passed"] for c in result["checks"]))


def all_configs_passed(results):
    """All-or-nothing: the test passes only if every configuration passed
    every one of its four file checks."""
    return all(config_passed(r) for r in results)


# Why one file check failed, in a word, for the per-configuration summary line.
CHECK_REASONS = {
    "missing_file": "missing",
    "unreadable": "unreadable",
    "content_mismatch": "mismatch",
}


def config_verdict(result):
    """The one-line PASS/FAIL summary of one configuration."""
    if result["failure"]:
        return "FAIL -- " + result["failure"].strip().splitlines()[0]
    checks = result["checks"]
    failed = [c for c in checks if not c["passed"]]
    if not failed:
        return "PASS (%d/%d files)" % (len(checks), len(checks))
    return "FAIL -- " + ", ".join(
        "%s %s" % (c["name"], CHECK_REASONS.get(c["error_type"], "failed"))
        for c in failed)


def format_config_detail(result):
    """The per-file breakdown of one configuration, with mismatch previews."""
    if result["failure"]:
        return result["failure"].rstrip()
    out = ["- %s: %s" % (c["name"], "PASS" if c["passed"] else "FAIL")
           for c in result["checks"]]
    for check in result["checks"]:
        if not check["passed"]:
            out.append("")
            out.append("--- %s ---" % check["name"])
            out.append(format_check_failure(check).rstrip())
    return "\n".join(out)


def format_assembler_output(name, results, picked):
    passed = all_configs_passed(results)
    total = len(results)
    score = POINTS_ASSEMBLER[name] if passed else Fraction(0)
    out = [
        "Test program: %s.s" % name,
        "Score: %.2f / %.2f" % (float(score), float(POINTS_ASSEMBLER[name])),
        "Passed %d / %d .data configurations."
        % (sum(1 for r in results if config_passed(r)), total),
        "",
        "Your assembler is run once per .data configuration of this program. "
        "This test is all-or-nothing: all four output files must match under "
        "EVERY configuration to earn this test's points.",
        "",
    ]
    out += ["- config %d/%d: %s" % (r["config"], total, config_verdict(r))
            for r in results]
    for result in results:
        if config_passed(result):
            continue
        out.append("")
        out.append("=== config %d/%d ===" % (result["config"], total))
        out.append(format_config_detail(result))
    out.append("")
    if picked:
        out.append(picked)
    out.append("Note: text-file comparisons ignore extra blank lines, "
               "leading/trailing spaces, uppercase/lowercase hex letters, "
               "and Windows/Linux line-ending differences.")
    return passed, "\n".join(out) + "\n"


def grade_assembler(name, key, found, all_files, chosen_dir):
    """Returns (passed, output) for one assembler sub-test."""
    if ("assembler", name) in key.errors:
        return False, internal_error_text(key.errors[("assembler", name)])

    assembler_py = found.get("assembler.py")
    if not assembler_py:
        return False, (
            "Could not find assembler.py in your submission.\n"
            "Upload your assembler as a file named exactly assembler.py "
            "(the folder it sits in does not matter).\n\n%s"
            % describe_submission(all_files)
        )

    picked = describe_choice(found, ["assembler.py"], chosen_dir)
    if time_left() < MIN_SUBTEST_SEC:
        return False, budget_exhausted_text() + picked

    configs = CONFIGS
    workdir = tempfile.mkdtemp(prefix="asm_%s_" % name)
    try:
        # Staged ONCE for the whole test: stage_student_assembler copies the
        # student's whole directory, and a second copy over the same
        # destination would silently fall back to the single-file path and
        # break a multi-file assembler.
        staged = stage_student_assembler(assembler_py, workdir)
        results = []
        for index, config in enumerate(configs):
            if time_left() < MIN_SUBTEST_SEC:
                results.append({"config": config, "checks": [],
                                "failure": budget_exhausted_text()})
                continue
            # Split what is left of the budget over the configurations this
            # test still has to run, so one slow configuration cannot eat the
            # whole allowance and leave the others unrun.
            share = time_left() / (len(configs) - index)
            rundir, failure = run_student_assembler(
                key, name, config, staged, workdir, share)
            checks = ([] if failure
                      else check_assembler_outputs(key, name, config, rundir))
            results.append({"config": config, "checks": checks,
                            "failure": failure})
    finally:
        keep_outputs("asm_%s" % name,
                     files=generated_assembler_outputs(workdir, name, configs))
        shutil.rmtree(workdir, ignore_errors=True)

    if len(configs) == 1 and results[0]["failure"]:
        # No configurations to compare across: the single run failing is the
        # whole story, so report it on its own rather than as a one-row table.
        return False, results[0]["failure"] + "\n" + picked
    return format_assembler_output(name, results, picked)


# --------------------------------------------------------------------------
# Results
# --------------------------------------------------------------------------

def existing_results_are_valid():
    """True if a parseable results.json is already on disk.

    Guards the non-atomic fallback below: a truncating rewrite that fails
    partway would replace a valid file with a corrupt one, which is worse than
    leaving the older scores in place.
    """
    try:
        with open(RESULTS_JSON) as f:
            json.load(f)
        return True
    except Exception:
        return False


def clear_results_obstruction():
    """Make the results path writable again if a student sabotaged it.

    A student's assembler can rmtree the results directory and recreate
    results.json as a DIRECTORY. os.replace onto a directory raises
    IsADirectoryError, and so does an in-place open, so every write path fails
    and all earned credit is lost. The same trick works on the results
    directory itself. Remove whatever is in the way -- guarded, like every
    other delete here, to something that resolves inside AUTOGRADER_ROOT.
    """
    real_root = os.path.realpath(AUTOGRADER_ROOT)

    def inside_root(path):
        try:
            real = os.path.realpath(path)
        except OSError:
            return False
        try:
            return (os.path.commonpath([real_root, real]) == real_root
                    and real != real_root)
        except ValueError:
            return False

    # The results directory replaced by a file (or a symlink to one).
    if os.path.exists(RESULTS_DIR) and not os.path.isdir(RESULTS_DIR):
        if inside_root(RESULTS_DIR):
            try:
                os.unlink(RESULTS_DIR)
            except Exception:
                pass

    # results.json replaced by a directory, socket, symlink, ...
    if os.path.lexists(RESULTS_JSON) and not os.path.isfile(RESULTS_JSON):
        if inside_root(RESULTS_JSON):
            try:
                if os.path.isdir(RESULTS_JSON) and not os.path.islink(RESULTS_JSON):
                    shutil.rmtree(RESULTS_JSON)
                else:
                    os.unlink(RESULTS_JSON)
            except Exception:
                pass


def write_results_atomic(payload):
    """Write results.json atomically, and never give up quietly.

    json.dumps happens first so a serialization failure cannot truncate a good
    file. The bytes go to a temp file in the same directory, are fsynced, and
    are then os.replace()d into place, so a failed or partial write leaves the
    previous valid results.json untouched instead of corrupting it.

    On failure the JSON is printed to stdout, where it at least reaches the
    Gradescope log.
    """
    try:
        text = json.dumps(payload, indent=4, allow_nan=False)
    except Exception:
        text = json.dumps({"score": 0,
                           "output": "Autograder could not serialize results.",
                           "tests": []}, indent=4)

    for attempt in range(2):
        tmp_path = None
        try:
            clear_results_obstruction()
            os.makedirs(RESULTS_DIR, exist_ok=True)
            # A student's assembler can chmod this directory read-only; take
            # it back before every write, not just after a failure.
            try:
                # 0700, NOT 0755: the image hardens this directory to
                # root-only and widening it would reopen the results-tampering
                # hole that privilege separation exists to close.
                os.chmod(RESULTS_DIR, 0o700)
            except Exception:
                pass
            fd, tmp_path = tempfile.mkstemp(dir=RESULTS_DIR,
                                            prefix=".results-", suffix=".tmp")
            with os.fdopen(fd, "w") as f:
                f.write(text)
                f.flush()
                os.fsync(f.fileno())
            # mkstemp creates 0600; Gradescope must be able to read it.
            try:
                os.chmod(tmp_path, 0o644)
            except Exception:
                pass
            os.replace(tmp_path, RESULTS_JSON)
            return True
        except Exception:
            if tmp_path:
                try:
                    os.unlink(tmp_path)
                except Exception:
                    pass

    # Last ditch: overwrite in place. This is the very non-atomic write that
    # can leave a truncated file, so it is only allowed when there is no valid
    # results.json to lose -- never trade a good file for a corrupt one.
    if not existing_results_are_valid():
        try:
            with open(RESULTS_JSON, "w") as f:
                f.write(text)
                f.flush()
                os.fsync(f.fileno())
            return True
        except Exception:
            pass

    sys.stdout.write(
        "\n===== AUTOGRADER COULD NOT WRITE %s =====\n%s\n===== END RESULTS =====\n"
        % (RESULTS_JSON, text))
    sys.stdout.flush()
    return False


def record(number, name, max_score, passed, output):
    """Record one finished sub-test. Every sub-test is all-or-nothing, so the
    verdict determines the score.

    `status` is carried alongside `score` because they are not the same fact: a
    0-point diagnostic test (addition) scores 0.00 / 0.00 whether it passed or
    failed, and `status` is the only field that tells the two apart. Gradescope
    renders it, and run_test.sh reads it for its PASS/FAIL summary.
    """
    score = max_score if passed else Fraction(0)
    # float(Fraction) with NO rounding: rounding each test to N places makes
    # the per-test values stop summing to the total (an even three-way split
    # rounds away from the program total), whereas the unrounded floats of these
    # particular fractions sum back to exactly 3.0. finalize() then adds up
    # these very numbers, so the top-level score always equals the sum of what
    # is actually in the file.
    RESULTS_BY_NUMBER[number] = {
        "number": number,
        "name": name,
        "score": float(score),
        "max_score": float(max_score),
        "status": "passed" if passed else "failed",
        "output": output,
        "visibility": "visible",
    }


def guarded(number, name, max_score, fn, *args):
    """Run one sub-test so that its failure cannot cost the others.

    Without this, a raise anywhere (shutil.copyfile, tempfile.mkdtemp,
    os.listdir, a decode) discards all partial credit computed so far.
    """
    try:
        passed, output = fn(*args)
    except InternalGradingError as exc:
        passed = False
        output = internal_error_text(str(exc))
    except Exception as exc:
        passed = False
        output = internal_error_text(
            "This test could not be run.\n\n%s: %s\n\n%s"
            % (type(exc).__name__, exc, traceback.format_exc(limit=6)))
    # Driven by the test's own max, never by its name, so a repoint that gives
    # this program credit (or takes credit away from another) stays correct.
    if max_score == 0:
        output = ZERO_POINT_NOTICE + "\n\n" + output
    record(number, name, max_score, passed, output)
    # Flush after every test so an OOM kill or a wall-clock kill still leaves
    # behind the credit earned up to this point.
    finalize()


def finalize(note=None):
    """Emit results.json from whatever has been recorded so far."""
    tests = []
    for number, name, max_score in TEST_SPECS:
        test = RESULTS_BY_NUMBER.get(number)
        if test is None:
            test = {
                "number": number,
                "name": name,
                "score": 0.0,
                "max_score": float(max_score),
                "status": "failed",
                "output": internal_error_text(
                    "This test did not run because the autograder stopped "
                    "early."),
                "visibility": "visible",
            }
        tests.append(test)

    total = sum(t["score"] for t in tests)
    max_total = sum(t["max_score"] for t in tests)
    output = "Phase 1 Autograder -- %.2f / %.2f\n" % (total, max_total)
    if note:
        output += "\n" + note
    return write_results_atomic({"score": total, "output": output,
                                 "tests": tests})


def refuse_to_grade(reason):
    """Emit a valid, schema-correct, all-zero results.json and grade nothing.

    Failing closed: no student process is spawned at all.
    """
    for number, name, max_score in TEST_SPECS:
        record(number, name, max_score, False,
               internal_error_text(
                   "This submission was not graded because the autograder "
                   "could not sandbox untrusted code.\n\n" + reason))
    banner = ("AUTOGRADER MISCONFIGURATION: could not drop privileges, "
              "refusing to run submissions as root - contact course staff")
    sys.stdout.write("\n*** %s ***\n%s\n" % (banner, reason))
    sys.stdout.flush()
    finalize(note=banner + "\n\n" + reason)


def main():
    # The answer key is read in full BEFORE any student code can run.
    # Must happen before any child is spawned, so that detached descendants
    # re-parent to us rather than to PID 1.
    become_subreaper()
    PRIVSEP.decide()
    sys.stdout.write(PRIVSEP.describe() + "\n")
    sys.stdout.flush()

    # Root without an armed sandbox is strictly worse than the native case:
    # student code could forge results.json and SIGKILL us. Grade nothing.
    if not PRIVSEP.can_grade():
        refuse_to_grade(PRIVSEP.refusal or PRIVSEP.reason)
        return

    key = preload_answer_key()

    # Only now that the whole key is in memory may it leave the disk, and only
    # then may any student process start.
    note = remove_on_disk_answer_key()
    sys.stdout.write(note + "\n")
    sys.stdout.flush()

    found, all_files, chosen_dir = discover_submission()

    for program in (ASSEMBLY_TESTS if "assembly" in PARTS else []):
        for config in CONFIGS:
            guarded(assembly_number(program, config),
                    assembly_test_name(program, config),
                    assembly_points(program),
                    grade_assembly, program, config, key, found, all_files,
                    chosen_dir)

    for name in (ASSEMBLER_TESTS if "assembler" in PARTS else []):
        guarded(ASSEMBLER_NUMBERS[name], "Assembler: %s" % name,
                POINTS_ASSEMBLER[name],
                grade_assembler, name, key, found, all_files, chosen_dir)

    # Nothing student-spawned may outlive the grader and rewrite results.json.
    strays = reap_stray_children()
    if strays:
        sys.stdout.write("killed %d stray student process(es)\n" % strays)
    finalize()


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:   # SystemExit/KeyboardInterrupt included
        try:
            finalize(note=(
                "The autograder stopped early with an unexpected internal "
                "error. Scores shown above are whatever completed before it.\n"
                "%s: %s\n\n%s"
                % (type(exc).__name__, exc, traceback.format_exc(limit=8))))
        except BaseException:
            try:
                sys.stdout.write("autograder failed catastrophically: %r\n" % (exc,))
            except Exception:
                pass
    sys.exit(0)
