# MIPS_project   
## Introduction   
This is the final project for the Digital Circuit Design course. The target is to implement a MIPS in verilog and synthesize the circuit. For detail information about the design, please refer to MIPS_intro.pdf and DSD_FinalDescription.pdf in /spec_and_materials.
## Design Details
Please refer to MIPS project.pdf for design details.  
The MIPS and cache implementation can be found in baseline/src/CHIP.v.  
The testbench is baseline/src/Final_tb.v.  

## Usage
For rtl-level simulation, in baseline/src:   
```
ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
```   
