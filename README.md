# MIPS_project   
## Introduction   
This was the final project for Digital Circuit Design. The target is to implement a MIPS processor. I rewrite the project to achive better performance and code readability. For details about the changes, please refer to MIPS_V2.pdf. DSD_FinalDescription.pdf defines the instructions supported as well as the evaluation metric.

## Design Details  
The MIPS and cache implementation can be found in baseline/src/CHIP.v.  
The testbench is baseline/src/Final_tb.v.  

## Usage
For rtl-level simulation, in src/:   
```
ncverilog Final_tb.v CHIP.v cache.v control.v mips.v slow_memory.v +define+hasHazard +access+r
```   
