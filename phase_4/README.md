# Phase 4 Documentation

You will find the documentation and problem descriptions for phase four in `phase_4/documentation/phase_4.pdf`. Be sure to **read all pages** of the PDF. There is one part to this phase, with possible extra credit points. You will also submit all previous files (i.e., `decoder.v`, `imm.v`, `alu.v`, `rf.v`).

1. Pipelined RV32I CPU (`hart.v`)

# Rubric
| Category | Points |
| -------- | -------- |
| Pipelined Processor    | 3    |
| Hazard Detection    | 2    |
| Forwarding Logic    | 2    |
| Total | 7 |


# Install

I recommend using conda, as this will allow you to install everything needed.

https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html

Once you have conda, follow the instructions below.

### Linux/MacOS

```
git clone https://github.com/UnaryLab/EEL4768_RISC-V_Project
cd EEL4768_RISC-V_Project/phase_4/
conda env create -f environment.yaml
```
Then, you can activate the conda environment
```
conda activate eel4768_phase_4
```
You need to reactivate or make sure you are in this conda env before running the test script everytime.

### Windows

Icarus and GTK cannot be directly installed through conda. If you have a windows system, I recommend either using [wsl](https://learn.microsoft.com/en-us/windows/wsl/install) (Windows subsystem for linux), or you can use the [eustis server](https://www.youtube.com/watch?v=KGm5RdI_gNA).

Both of these solutions will run a linux operating system. If you have issues, please come to my office hours.

# Testing your work

**There is no autograder in this repository.** Verifying that your `hart.v`,
`alu.v`, `imm.v`, `rf.v` and `decoder.v` behave correctly is part of the assignment.
Follow the example testbench outlined in `phase_2/example/` to understand how to write a testbench.

## The example

`phase_2/example/` holds two files:

- **`opmux.v`** -- a small combinational module: four inputs (`i_a`, `i_b`,
  `i_sel`, `i_en`), two outputs (`o_result`, `o_zero`), and a two-bit select
  choosing between `+`, `-`, `<<` and `>>`.
- **`opmux_tb.v`** -- a self-checking testbench for it. **This is the file to
  read.** It is commented as a walkthrough, and its structure is the one every
  testbench you write this semester will have.

## The traces

`phase_4/traces/` holds two programs for your hart to run and the trace it should
produce:

- **`no_hazard_program.hex`**: Does not consider hazards.
- **`hazard_program.hex`**: Considers hazards.

There are then 3 files for expected outputs.

- **`no_hazard.trace`**: Expected output for `no_hazard_program.hex`
- **`hazard_no_fwd.trace`**: Expected output for `hazard_program.hex` ***with no*** forwarding logic.
- **`hazard_no_fwd.trace`**: Expected output for `hazard_program.hex` ***with*** forwarding logic.

## Run iverilog

To run the example testbench, follow the script below. To run your own verilog file and testbench, just replace the paths for the testbench and target file.

```
cd EEL4768_RISC-V_Project/phase_2/
iverilog -s opmux_tb -o sim example/opmux_tb.v example/opmux.v
./sim
```

For phase 4, list your testbench and every file your hart needs:

```
iverilog -s hart_tb -o sim hart_tb.v hart.v alu.v imm.v rf.v decoder.v
./sim
```

`-s` names the top module to elaborate, `-o` names the simulator to write, and
every source file the design needs is listed after them.

Running the example prints:

```
========== opmux testbench ==========
--- add ---
[PASS] add: 7 + 9
[PASS] add: 0 + 0 sets zero
...
212 passed, 0 failed
ALL TESTS PASSED
```

## Waveforms

The example also writes `opmux.vcd`:

```
gtkwave opmux.vcd
```

Add `$dumpfile`/`$dumpvars` to your own testbench the same way to get a waveform of your hart.

This outputs a waveform, similar to the ones from digital systems, to view. This is helpfull for debugging.
