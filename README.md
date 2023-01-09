# MIPS_project   
## Introduction   
This was the final project for Digital Circuit Design. The target is to implement a 5-stage MIPS processor. I rewrote the project to achieve better performance and code readability. For details about the changes, please refer to MIPS_V2.pdf. DSD_FinalDescription.pdf defines the instructions supported as well as the evaluation metric. The previous version of MIPS is provided in the V1 folder.   

## Usage
For rtl-level simulation, in the src folder:   
```
ncverilog Final_tb.v CHIP.v cache.v control.v mips.v slow_memory.v +define+hasHazard +access+r
```   
