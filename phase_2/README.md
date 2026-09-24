# Phase 2 Documentation

**This directory is a full test of your phase_2 to follow the small changes outlined in phase_3. I built this as a full test harness to help debug your code.**

You will find the documentation and problem descriptions for phase two in `phase_2/documentation/phase_2.pdf`. Be sure to **read all pages** of the PDF. There are four parts to this phase, that break down as follows.

1. ALU

2. Immediate Generator

3. Register File

4. Instruction Decoder

`phase_2/skeletons/` has a starting point for each of the four: the module
header and the full port list, documented port by port, with the body left for
you.

# Install

I recommend using conda, as this will allow you to install everything needed.

https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html

Once you have conda, follow the instructions below.

### Linux/MacOS

```
git clone https://github.com/UnaryLab/EEL4768_RISC-V_Project
cd EEL4768_RISC-V_Project/phase_2/
conda env create -f environment.yaml
```
Then, you can activate the conda environment
```
conda activate eel4768_phase_2
```
You need to reactivate or make sure you are in this conda env before running the test script everytime.

### Windows

Icarus and GTK cannot be directly installed through conda. If you have a windows system, I recommend either using [wsl](https://learn.microsoft.com/en-us/windows/wsl/install) (Windows subsystem for linux), or you can use the [eustis server](https://www.youtube.com/watch?v=KGm5RdI_gNA).

Both of these solutions will run a linux operating system. If you have issues, please come to my office hours.

# Phase 2 grader

## Setup

With the conda environment active, install the Verilog rule checker once:

```
pip install phase_2/source/python-ece552/
```

## Put your files here

```
phase_2/submission/
    alu.v
    decoder.v
    imm.v
    rf.v
```

## Run it

From the `EEL4768_RISC-V_Project/` folder:

```
./phase_2/scripts/student_test.sh
```

Pass a directory to check files kept somewhere other than
`phase_2/submission/`:

```
./phase_2/scripts/student_test.sh /path/to/your/verilog
```

## Reading the output

The script prints each test's score, a total out of 3, and the output of any
failed test. The full log is saved to `phase_2/output/log.txt`.