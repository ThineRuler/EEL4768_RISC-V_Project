 ### Verification Results

  #### 1. Linter Verification

    ece552 validate -w test/cache.v
    # Output: Exit code 0 (Passes all synthesizability and course rule checks)

  #### 2. Testbench Verification (test/cache_tb.v)

  To run the testbench:

    iverilog -g2005 -s cache_tb -o sim test/cache_tb.v test/cache.v
    ./sim