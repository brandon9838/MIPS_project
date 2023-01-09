module MIPS_Pipeline(
	// control interface
		clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata
	);

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

//==== input/output ============================
	input clk, rst_n;
	
	output ICACHE_ren, ICACHE_wen;
	output [29:0] ICACHE_addr;
	output [31:0] ICACHE_wdata;
	input  ICACHE_stall;
	input  [31:0] ICACHE_rdata;
	
	output DCACHE_ren, DCACHE_wen;
	output [29:0] DCACHE_addr;
	output [31:0] DCACHE_wdata;
	input  DCACHE_stall;
	input  [31:0] DCACHE_rdata;
	
	
//==== wire/reg ================================
	//IF stage
	reg [31:0] PC_IF_r,PC_IF_w;
	
	//IFID stage
	reg [31:0] instruction_IFID_r,instruction_IFID_w;
	reg [31:0] PC4_IFID_r,PC4_IFID_w;
	reg [31:0] PC4_IF;
	reg [31:0] j_addr_IF;
	
	//IDEX stage
	reg [31:0] PC4_IDEX_r,PC4_IDEX_w;
	reg [5:0]  opcode_IDEX_r,opcode_IDEX_w;
	reg [31:0] rs_data_IDEX_r,rs_data_IDEX_w;
	reg [31:0] rt_data_IDEX_r,rt_data_IDEX_w;
	reg [15:0] imm_IDEX_r,imm_IDEX_w;
	reg [4:0]  reg_write_addr_IDEX_r,reg_write_addr_IDEX_w; //will be used when passed to wb stage, not in id stage
		
		//Register file
		reg [31:0]register_r[31:0],register_w[31:0];
		reg [4:0] reg_rs_addr_IFID, reg_rt_addr_IFID;
		reg [31:0]reg_rs_data_IFID, reg_rt_data_IFID;
		
		// MIPS control output signal
		wire ctrl_j_IF;
		
		wire ctrl_jlink_IFID;
		wire ctrl_regdst_IFID;
		
		wire [1:0] ctrl_aluop_IDEX;
		wire ctrl_alusrc_IDEX;
		wire ctrl_flush_IDEX;
		wire ctrl_imm_zero_extd_IDEX;
		wire ctrl_reg_write_IDEX;
		wire ctrl_beq_taken_IDEX;
		wire ctrl_jr_IDEX;
		
		wire ctrl_lw_EXMEM;
		wire ctrl_sw_EXMEM;
		wire ctrl_reg_write_EXMEM;
		
		wire ctrl_reg_write_MEMWB;
		
		wire ctrl_mem_stall;
		wire ctrl_loaduse_bubble;	
	
	//EXMEM	
	reg [4:0]  reg_write_addr_EXMEM_r, reg_write_addr_EXMEM_w;
	reg [31:0] reg_write_data_EXMEM_r, reg_write_data_EXMEM_w;
	reg [31:0] sw_rt_data_EXMEM_r, sw_rt_data_EXMEM_w; //data saved to mem when sw
	
	reg [31:0] imm_sign_ext_IDEX, imm_zero_ext_IDEX;
	reg [31:0] beq_addr_IDEX, jr_addr_IDEX;
		
		//ALU
		reg [31:0] ALU_in1, ALU_in2;
		reg [31:0] ALU_add, ALU_sub, ALU_and, ALU_or, ALU_xor;
		reg [4:0] ALU_shamt;
		reg [5:0] ALU_func;
		reg [31:0] ALU_out;
		reg ALU_zero;
	
	//MEMWB
	reg [4:0] reg_write_addr_MEMWB_r,reg_write_addr_MEMWB_w;
	reg [31:0] reg_write_data_MEMWB_r,reg_write_data_MEMWB_w;
	
	integer i;
//====  IF stage combinational/sequential circuit ==============================
	//I cache combinational circuit
	assign ICACHE_ren=1'b1;
	assign ICACHE_wen=1'b0;
	assign ICACHE_wdata=32'd0;
	assign ICACHE_addr=PC_IF_r[31:2];
	
	always@(*)begin //Combinational circuit 
		if(ctrl_mem_stall || ctrl_loaduse_bubble)	PC_IF_w=PC_IF_r;
		else if (ctrl_j_IF)							PC_IF_w=j_addr_IF;
		else if (ctrl_beq_taken_IDEX)				PC_IF_w=beq_addr_IDEX;
		else if (ctrl_jr_IDEX)						PC_IF_w=jr_addr_IDEX;
		else 										PC_IF_w=PC4_IF;
	end                                 

	always@( posedge clk or negedge rst_n ) begin	//Sequential circuit
		if(!rst_n) PC_IF_r<=0;
		else 	PC_IF_r<=PC_IF_w;
	end
	
//====  IFID stage combinational/sequential circuit ==============================
	always@(*)begin //Combinational circuit
		PC4_IF=PC_IF_r+4;
		j_addr_IF={PC_IF_r[31:28], ICACHE_rdata[25:0], 2'b0};	//jump address for j/jal
		
		if (ctrl_mem_stall || ctrl_loaduse_bubble)begin
			instruction_IFID_w=instruction_IFID_r;
			PC4_IFID_w=PC4_IFID_r;
		end
		else if (ctrl_flush_IDEX)begin
			instruction_IFID_w=32'h00000000;
			PC4_IFID_w=PC4_IF;
		end
		else begin
			instruction_IFID_w=ICACHE_rdata;
			PC4_IFID_w=PC4_IF;
		end
	end
		
	always@( posedge clk or negedge rst_n ) begin	//Sequential circuit
		if(!rst_n)begin
			instruction_IFID_r<=32'h00000000;
			PC4_IFID_r<=32'h00000000;
		end
		else begin
			instruction_IFID_r<=instruction_IFID_w;
			PC4_IFID_r<=PC4_IFID_w;
		end
	end
	
//====  Register file combinational/sequential circuit ==============================	
	always@(*)begin	//Combinational circuit
		reg_rs_addr_IFID=instruction_IFID_r[25:21];
		reg_rt_addr_IFID=instruction_IFID_r[20:16];
		reg_rs_data_IFID=register_r[reg_rs_addr_IFID];
		reg_rt_data_IFID=register_r[reg_rt_addr_IFID];

		for (i=0;i<32;i=i+1) 		register_w[i]=register_r[i];
		if (ctrl_reg_write_MEMWB)	register_w[reg_write_addr_MEMWB_r]=reg_write_data_MEMWB_r;

	end
	
	always@( posedge clk or negedge rst_n ) begin	//Sequential circuit
		if(!rst_n)begin
			register_r[0]<=32'h00000000;
			for (i=1;i<32;i=i+1) register_r[i]<=32'h00000000;
		end
		else begin
			register_r[0]<=32'h00000000;
			for (i=1;i<32;i=i+1) register_r[i]<=register_w[i];
		end
	end
	
//====  IDEX stage combinational/sequential circuit ==============================	
	always@(*)begin //Rs combinational circuit(forwarding)
		if (ctrl_mem_stall)										rs_data_IDEX_w=rs_data_IDEX_r;
		else if(reg_write_addr_IDEX_r==reg_rs_addr_IFID && 
			ctrl_reg_write_IDEX && 
			reg_write_addr_IDEX_r!=5'b00000)					rs_data_IDEX_w=ALU_out;	
		else if(reg_write_addr_EXMEM_r==reg_rs_addr_IFID && 
			ctrl_reg_write_EXMEM && 
			reg_write_addr_EXMEM_r!=5'b00000)					rs_data_IDEX_w=reg_write_data_MEMWB_w;
																//cannot use reg_write_data_EXMEM_r because it could be a lw 
		else if(reg_write_addr_MEMWB_r==reg_rs_addr_IFID && 
			ctrl_reg_write_MEMWB && 
			reg_write_addr_MEMWB_r!=5'b00000)					rs_data_IDEX_w=reg_write_data_MEMWB_r;
			
		else if(ctrl_jlink_IFID)								rs_data_IDEX_w=PC4_IFID_r;		//save PC4 when jal or jalr
		else 													rs_data_IDEX_w=reg_rs_data_IFID;
	end
	
	always@(*)begin //Rt combinational circuit(forwarding)
		if (ctrl_mem_stall)										rt_data_IDEX_w=rt_data_IDEX_r;
		else if(reg_write_addr_IDEX_r==reg_rt_addr_IFID && 
			ctrl_reg_write_IDEX && 
			reg_write_addr_IDEX_r!=5'b00000)					rt_data_IDEX_w=ALU_out;	
		else if(reg_write_addr_EXMEM_r==reg_rt_addr_IFID && 
			ctrl_reg_write_EXMEM && 
			reg_write_addr_EXMEM_r!=5'b00000)					rt_data_IDEX_w=reg_write_data_MEMWB_w;
																//cannot use reg_write_data_EXMEM_r because it could be a lw 
		else if(reg_write_addr_MEMWB_r==reg_rt_addr_IFID && 
			ctrl_reg_write_MEMWB && 
			reg_write_addr_MEMWB_r!=5'b00000)					rt_data_IDEX_w=reg_write_data_MEMWB_r;
			
		else if(ctrl_jlink_IFID)								rt_data_IDEX_w=32'h00000000;	
																//save PC4 when jal or jalr, alu will sub rs(PC4) by rt(0)
		else 													rt_data_IDEX_w=reg_rt_data_IFID;
	end
	
	always@(*)begin //Combinational circuit
		if(ctrl_mem_stall)begin 
			PC4_IDEX_w=PC4_IDEX_r;
			opcode_IDEX_w=opcode_IDEX_r;
			imm_IDEX_w=imm_IDEX_r;
			reg_write_addr_IDEX_w=reg_write_addr_IDEX_r;
		end
		else begin
			PC4_IDEX_w=PC4_IFID_r;
			opcode_IDEX_w=instruction_IFID_r[31:26];
			imm_IDEX_w=instruction_IFID_r[15:0];
			reg_write_addr_IDEX_w=	(ctrl_regdst_IFID)?instruction_IFID_r[15:11]:	//if rtype, save to rd (including jalr)
									(ctrl_jlink_IFID)?5'b11111:		//save PC4 to &31 when jal (jalr saves PC4 to rd)
									instruction_IFID_r[20:16];		//else use rt
		end
	end
	
	always@( posedge clk or negedge rst_n ) begin	//Sequential circuit
		if(!rst_n)begin
			PC4_IDEX_r<=32'h00000000;
			opcode_IDEX_r<=6'b000000;
			rs_data_IDEX_r<=32'h00000000;
			rt_data_IDEX_r<=32'h00000000;
			imm_IDEX_r<=16'h0000;
			reg_write_addr_IDEX_r<=5'b00000;
		end
		else begin
			PC4_IDEX_r<=PC4_IDEX_w;
			opcode_IDEX_r<=opcode_IDEX_w;
			rs_data_IDEX_r<=rs_data_IDEX_w;
			rt_data_IDEX_r<=rt_data_IDEX_w;
			imm_IDEX_r<=imm_IDEX_w;
			reg_write_addr_IDEX_r<=reg_write_addr_IDEX_w;
		end
	end
//====  EXMEM stage combinational/sequential circuit ==============================	
	always@(*)begin //ALU combinational circuit
		//ALU input assignment
		ALU_shamt=imm_IDEX_r[10:6];
		ALU_func =imm_IDEX_r[5:0];
		ALU_in1=rs_data_IDEX_r;
		if(ctrl_alusrc_IDEX) 				ALU_in2=rt_data_IDEX_r; 
		else if (ctrl_imm_zero_extd_IDEX)	ALU_in2=imm_zero_ext_IDEX;
		else								ALU_in2=imm_sign_ext_IDEX;
		
		//ALU operations
		ALU_add=ALU_in1 + ALU_in2;
		ALU_sub=ALU_in1 - ALU_in2;
		ALU_and=ALU_in1 & ALU_in2;
		ALU_or =ALU_in1 | ALU_in2;
		ALU_xor=ALU_in1 ^ ALU_in2;
		
		ALU_zero=!(|ALU_sub);
		
		//ALU output MUX
		case(ctrl_aluop_IDEX)
			2'b00:begin	//Itype
				if (opcode_IDEX_r==ADDI) 		ALU_out=ALU_add;
				else if (opcode_IDEX_r==ANDI)	ALU_out=ALU_and;
				else if (opcode_IDEX_r==ORI)	ALU_out=ALU_or;
				else if (opcode_IDEX_r==XORI)	ALU_out=ALU_xor;
				else if (opcode_IDEX_r==SLTI)	ALU_out=ALU_sub;
				else							ALU_out=ALU_add; // addi/lw/sw
			end
			2'b10:begin //Rtype
				if (ALU_func==ADD)			ALU_out=ALU_add;
				else if (ALU_func==AND)		ALU_out=ALU_and;
				else if (ALU_func==OR)		ALU_out=ALU_or;
				else if (ALU_func==XOR)		ALU_out=ALU_xor;
				else if (ALU_func==NOR)		ALU_out=~ALU_or;
				else if (ALU_func==SLL)		ALU_out=ALU_in1 << ALU_shamt;
				else if (ALU_func==SRA)		ALU_out=ALU_in1 >>> ALU_shamt;
				else if (ALU_func==SRL)		ALU_out=ALU_in1 >> ALU_shamt;
				else if (ALU_func==SLT)		ALU_out=ALU_sub[31];
				else						ALU_out=ALU_sub; // sub/jalr
			end
			default:						ALU_out=ALU_sub; // beq/jal
		endcase
	end
	
	always@(*)begin //Combinational circuit
		imm_sign_ext_IDEX={{16{imm_IDEX_r[15]}}, imm_IDEX_r};
		imm_zero_ext_IDEX={16'h0000,imm_IDEX_r};
		beq_addr_IDEX={{14{imm_IDEX_r[15]}}, imm_IDEX_r,2'b00} + PC4_IDEX_r;
		jr_addr_IDEX=rs_data_IDEX_r;
		
		if (ctrl_mem_stall)begin
			reg_write_addr_EXMEM_w=reg_write_addr_EXMEM_r;
			reg_write_data_EXMEM_w=reg_write_data_EXMEM_r;
			sw_rt_data_EXMEM_w=sw_rt_data_EXMEM_r;
		end
		else begin
			reg_write_addr_EXMEM_w=reg_write_addr_IDEX_r;
			reg_write_data_EXMEM_w=ALU_out;
			sw_rt_data_EXMEM_w=rt_data_IDEX_r;
		end
	end
	
	always@( posedge clk or negedge rst_n ) begin	//Sequential circuit
		if(!rst_n)begin
			reg_write_addr_EXMEM_r<=5'b00000;
			reg_write_data_EXMEM_r<=32'h00000000;
			sw_rt_data_EXMEM_r<=32'h00000000;
		end
		else begin
			reg_write_addr_EXMEM_r<=reg_write_addr_EXMEM_w;
			reg_write_data_EXMEM_r<=reg_write_data_EXMEM_w;
			sw_rt_data_EXMEM_r<=sw_rt_data_EXMEM_w;
		end
	end
//====  MEMWB stage combinational/sequential circuit ==============================
	//D cache combinational circuit
	assign DCACHE_ren=ctrl_lw_EXMEM;
	assign DCACHE_wen=ctrl_sw_EXMEM;
	assign DCACHE_addr=reg_write_data_EXMEM_r[31:2];	//ALU output, lw/sw address is calculated by ALU
	assign DCACHE_wdata=sw_rt_data_EXMEM_r;		//save rt when sw
	
	always@(*)begin //Combinational circuit
		if (ctrl_mem_stall)begin
			reg_write_addr_MEMWB_w=reg_write_addr_MEMWB_r;
			reg_write_data_MEMWB_w=reg_write_data_MEMWB_r;
		end
		else begin
			reg_write_addr_MEMWB_w=reg_write_addr_EXMEM_r;
			reg_write_data_MEMWB_w=(ctrl_lw_EXMEM)? DCACHE_rdata:
									reg_write_data_EXMEM_r;
		end
	end
	
	always@( posedge clk or negedge rst_n ) begin	//Sequential circuit
		if(!rst_n)begin
			reg_write_addr_MEMWB_r<=5'b00000;
			reg_write_data_MEMWB_r<=32'h00000000;
		end
		else begin
			reg_write_addr_MEMWB_r<=reg_write_addr_MEMWB_w;
			reg_write_data_MEMWB_r<=reg_write_data_MEMWB_w;
		end
	end
	
//==== Control unit ==============================
	control mips_control(
		clk,						
		rst_n,	
		
		//input
		instruction_IFID_w [31:26],	//ctrl_opcode_IF
		
		instruction_IFID_r [31:26],	//ctrl_opcode_IFID,
		instruction_IFID_r [25:21],	//ctrl_rs_addr_IFID,
		instruction_IFID_r [20:16],	//ctrl_rt_addr_IFID,
		instruction_IFID_r [5:0],	//ctrl_func_IFID,
		
		reg_write_addr_IDEX_r,		//ctrl_reg_write_addr_IDEX
		ALU_zero,					//ctrl_zero_IDEX,
		
		ICACHE_stall,
		DCACHE_stall,
		
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
endmodule