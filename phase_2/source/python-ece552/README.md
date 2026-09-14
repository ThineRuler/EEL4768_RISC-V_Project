# ece552

The same Verilog rule checker Gradescope runs on your submission, so you
can check your own work *before* submitting instead of finding out from a
failed autograder run. See [Verilog Rules](../rules) on the course site
for what it actually checks (banned operators, disallowed constructs,
etc.) -- this tool enforces that page, it doesn't duplicate its
explanation.

## Install

Requires Python 3.10-3.13 (this tool depends on `pyslang`, which as of
version 9.1.0 doesn't ship a prebuilt wheel for anything newer -- if your
system's default `python3` is newer, e.g. 3.14, create a 3.10-3.13 venv
first: `python3.10 -m venv ~/.venvs/ece552 && ~/.venvs/ece552/bin/pip
install ./python-ece552/`, then use that venv's `ece552` binary below).

```sh
pip install ./python-ece552/
```

## Use

```sh
ece552 validate -w yourfile.v another.v ...
```

Run it against every `.v` file in your submission before you upload to
Gradescope. Exit code 0 means it passed; any output means something in
your submission violates a rule in [Verilog Rules](../rules) -- read the
error, it points at the exact line and names the specific rule.

**This tool is not the same thing as `iverilog`.** Your code can compile
fine under `iverilog` and still fail here -- that's expected, not a bug.
This checker enforces the course's synthesizable-subset rules on top of
what a compiler considers valid Verilog. If it's complaining about code
that looks correct to you, re-read the relevant section of
[Verilog Rules](../rules) first; if you still think it's wrong, post on
Piazza or contact the teaching staff (per that page's own note, false
positives are possible).
