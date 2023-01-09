module control(
    //input
	clk,
    rst_n,
	
    ctrl_opcode_IF,
	
	ctrl_opcode_IFID,
	ctrl_rs_addr_IFID,
	ctrl_rt_addr_IFID,
	ctrl_func_IFID,
	
	ctrl_reg_write_addr_IDEX,
	ctrl_zero_IDEX,
	
	I_mem_stall,
	D_mem_stall,
	//output
	ctrl_j_IF,
	
	ctrl_jlink_IFID,
	ctrl_regdst_IFID,
	
	ctrl_aluop_IDEX,
	ctrl_alusrc_IDEX,
	ctrl_flush_IDEX,
	ctrl_imm_zero_extd_IDEX,
	ctrl_reg_write_IDEX,
	ctrl_beq_taken_IDEX,
	ctrl_jr_IDEX,
	
	ctrl_lw_EXMEM,
	ctrl_sw_EXMEM,
	ctrl_reg_write_EXMEM,

	ctrl_reg_write_MEMWB,
	
	ctrl_mem_stall,
	ctrl_loaduse_bubble
	
);
/*
	Variable naming:
	Suffix IFID means the signal changes according to the registers between the IF and ID pipeline stage.
*/

//==== parameters ============================
    //opcode encoding
	parameter RTYPE	= 6'b000000;
	parameter ADDI	= 6'b001000;
	parameter ANDI	= 6'b001100;
	parameter ORI	= 6'b001101;
	parameter XORI	= 6'b001110;
	parameter SLTI	= 6'b001010;
	parameter LW	= 6'b100011;
	parameter SW	= 6'b101011;
    parameter BEQ	= 6'b000100;
    parameter J 	= 6'b000010;
	parameter JAL 	= 6'b000011;
	
	//RTYPE func encoding
	parameter ADD	= 6'b100000;
	parameter SUB	= 6'b100010;
	parameter AND	= 6'b100100;
	parameter OR	= 6'b100101;
	parameter XOR	= 6'b100110;
	parameter NOR	= 6'b100111;
	parameter SLL	= 6'b000000;
	parameter SRA	= 6'b000011;
	parameter SRL	= 6'b000010;
	parameter SLT	= 6'b101010;
	parameter JR	= 6'b001000;
	parameter JALR	= 6'b001001;
	parameter NOP	= 6'b000000;
	
	
//==== input/output definition ===============
	//input
	input clk, rst_n;
	input [5:0] ctrl_opcode_IF;
	
	input [5:0] ctrl_opcode_IFID;
	input [4:0] ctrl_rs_addr_IFID, ctrl_rt_addr_IFID;
	input [5:0] ctrl_func_IFID;
	
	input [4:0] ctrl_reg_write_addr_IDEX;	//to detect load use bubble
	input ctrl_zero_IDEX;
	
	input I_mem_stall;
	input D_mem_stall;
	
	//output
	output reg ctrl_j_IF;
		   
	output reg ctrl_jlink_IFID;
	output reg ctrl_regdst_IFID;
		
	output reg [1:0] ctrl_aluop_IDEX;
	output reg ctrl_alusrc_IDEX;
	output reg ctrl_flush_IDEX;
	output reg ctrl_imm_zero_extd_IDEX;
	output reg ctrl_reg_write_IDEX;
	output reg ctrl_beq_taken_IDEX;
	output reg ctrl_jr_IDEX;
		
	output reg ctrl_lw_EXMEM;
	output reg ctrl_sw_EXMEM;
	output reg ctrl_reg_write_EXMEM;
		
	output reg ctrl_reg_write_MEMWB;
	
	output reg ctrl_mem_stall;
	output reg ctrl_loaduse_bubble;
	

//==== wire/reg definition ================================
	reg [8:0] ctrl_signal_IDEX_r,ctrl_signal_IDEX_w;
	reg [2:0] ctrl_signal_EXMEM_r,ctrl_signal_EXMEM_w;
	reg ctrl_signal_MEMWB_r,ctrl_signal_MEMWB_w;
	
	wire itype_IFID;

	
//==== combinational circuit ==============================

	assign itype_IFID=(ctrl_opcode_IFID== ADDI 	||
						ctrl_opcode_IFID== ANDI ||
						ctrl_opcode_IFID== ORI 	||
						ctrl_opcode_IFID== XORI ||
						ctrl_opcode_IFID== SLTI ||
						ctrl_opcode_IFID== LW	||
						ctrl_opcode_IFID== SW);
						
	
	always@(*)begin	// IFID stage combinational circuit
		ctrl_signal_IDEX_w[0]=!(ctrl_opcode_IFID==J || ctrl_opcode_IFID==SW || ctrl_opcode_IFID==BEQ || 
								ctrl_opcode_IFID==RTYPE && (ctrl_func_IFID==JR || ctrl_func_IFID==NOP));						
							//write to register: jump, sw, beq, jr, nop does not need to write to registers
							
		ctrl_signal_IDEX_w[1]=ctrl_opcode_IFID==SW;		//SW
		ctrl_signal_IDEX_w[2]=ctrl_opcode_IFID==LW;		//LW
		ctrl_signal_IDEX_w[3]=!itype_IFID ;				//use rt as ALU input 2		
		ctrl_signal_IDEX_w[4]=ctrl_opcode_IFID==BEQ;	//BEQ
		ctrl_signal_IDEX_w[5]=ctrl_opcode_IFID==RTYPE && (ctrl_func_IFID==JR || ctrl_func_IFID==JALR);
							//use the register output as the next PC value, only when jr or jalr
		
		ctrl_signal_IDEX_w[6]=(ctrl_opcode_IFID==ANDI || ctrl_opcode_IFID==ORI || ctrl_opcode_IFID==XORI);
							//logical extend the immediate field: only when doing logical immediate operations
		
		ctrl_signal_IDEX_w[8:7]=(ctrl_opcode_IFID==RTYPE)?2'b10:
								(ctrl_opcode_IFID==BEQ)?2'b01:
								(itype_IFID)?2'b00:
								2'b11;					//ALUop
		
		if (ctrl_mem_stall) ctrl_signal_IDEX_w=ctrl_signal_IDEX_r;
		else if (ctrl_flush_IDEX || ctrl_loaduse_bubble)begin
			ctrl_signal_IDEX_w[2:0]=3'b000;
			ctrl_signal_IDEX_w[5:4]=2'b00;
		end
	end
	
	
	always@(*)begin // IDEX/EXMEM stage combinational circuit
		ctrl_signal_EXMEM_w=ctrl_signal_IDEX_r[2:0];
		ctrl_signal_MEMWB_w=ctrl_signal_EXMEM_r[0];
		if (ctrl_mem_stall)begin
			ctrl_signal_EXMEM_w=ctrl_signal_EXMEM_r;
			ctrl_signal_MEMWB_w=ctrl_signal_MEMWB_r;
		end
	end
	
	
	always@(*)begin // Output assignment
		ctrl_j_IF=(ctrl_opcode_IF==J || ctrl_opcode_IF==JAL);
		//jump to the address specified
		
		ctrl_jlink_IFID	=(ctrl_opcode_IFID==JAL || ctrl_opcode_IFID==JALR);
		//replace rd, rt, rs with PC4, $31  
		ctrl_regdst_IFID=(ctrl_opcode_IFID==RTYPE);
		//select rd as register save destination
		
		ctrl_reg_write_IDEX=ctrl_signal_IDEX_r[0];
		ctrl_alusrc_IDEX=ctrl_signal_IDEX_r[3];
		ctrl_beq_taken_IDEX=(ctrl_signal_IDEX_r[4] && ctrl_zero_IDEX);
		//branch taken
		ctrl_jr_IDEX=ctrl_signal_IDEX_r[5];
		ctrl_flush_IDEX=ctrl_beq_taken_IDEX || ctrl_signal_IDEX_r[5];
		//flush ID stage and EX stage in the next cycle, when branch taken or jump register
		ctrl_imm_zero_extd_IDEX=ctrl_signal_IDEX_r[6];
		ctrl_aluop_IDEX=ctrl_signal_IDEX_r[8:7];
		
		ctrl_lw_EXMEM=ctrl_signal_EXMEM_r[2];
		ctrl_sw_EXMEM=ctrl_signal_EXMEM_r[1];
		ctrl_reg_write_EXMEM=ctrl_signal_EXMEM_r[0];
		
		ctrl_reg_write_MEMWB=ctrl_signal_MEMWB_r;
		
		//Signals that depends on multiple/none of the pipeline stages
		ctrl_mem_stall=I_mem_stall || D_mem_stall;
		ctrl_loaduse_bubble = ctrl_signal_IDEX_r[2] && 
							(ctrl_rs_addr_IFID== ctrl_reg_write_addr_IDEX || ctrl_rt_addr_IFID== ctrl_reg_write_addr_IDEX) && 
							(ctrl_reg_write_addr_IDEX!=5'b00000);
							/* 1.load is in EX stage
							2. load use
							3. write back destination is not 0*/
	end
	
	
//==== sequential circuit =================================
	always@( posedge clk or negedge rst_n ) begin
		if(!rst_n) begin
			ctrl_signal_IDEX_r<=9'b000000000;
			ctrl_signal_EXMEM_r<=3'b000;
			ctrl_signal_MEMWB_r<=1'b0;
		end
		else begin 
			ctrl_signal_IDEX_r<=ctrl_signal_IDEX_w;
			ctrl_signal_EXMEM_r<=ctrl_signal_EXMEM_w;
			ctrl_signal_MEMWB_r<=ctrl_signal_MEMWB_w;
		end
	end
endmodule