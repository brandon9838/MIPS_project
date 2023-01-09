// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	MIPS_Pipeline i_MIPS(
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
	
	cache D_cache(
		clk,
		~rst_n,
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata,
		mem_read_D,
		mem_write_D,
		mem_addr_D,
		mem_rdata_D,
		mem_wdata_D,
		mem_ready_D
	);

	cache I_cache(
		clk,
		~rst_n,
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
		mem_read_I,
		mem_write_I,
		mem_addr_I,
		mem_rdata_I,
		mem_wdata_I,
		mem_ready_I
	);
endmodule

module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_wdata,
    proc_stall,
    proc_rdata,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output   reg   proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output   reg   mem_read, mem_write;
    output  reg [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    //proc_tag = proc_addr[29:5],  proc_index = proc_addr[4:2], offset = proc_addr[1:0]
    //write_back = block_match[154] , valid= block_match[153], cache_tag = block_match[152:128],
    parameter IDLE          = 2'b00;
    parameter WRITE_BACK    = 2'b01;
    parameter READ_FROM_MEM = 2'b10;

    reg [1:0] state, state_next;
    reg [153:0] blocks[7:0];
    reg [153:0] blocks_next[7:0];
    wire [24:0] proc_tag;
    wire [2:0]  proc_index;
    wire [1:0]  offset;
    wire [153:0] block_match;
    wire valid,hit;
    wire [24:0] cache_tag;
    integer i;
    
//==== combinational circuit ==============================
    assign proc_tag = proc_addr[29:5];
    assign proc_index = proc_addr[4:2];
    assign offset = proc_addr[1:0];
    assign block_match = blocks[proc_index];
    assign proc_rdata = (offset == 2'b00)? block_match[31:0]:
                        (offset == 2'b01)? block_match[63:32]:
                        (offset == 2'b10)? block_match[95:64]: block_match[127:96];
    
    assign valid  = block_match[153];
    assign hit = (proc_tag == cache_tag) ? 1'b1: 1'b0;
    assign cache_tag = block_match[152:128];
    assign mem_wdata = block_match[127:0];

    always@(*)begin   // set memory-related signal
        case(state)
            IDLE:
            begin
                mem_read = 1'b0;
                mem_write= 1'b0;
                mem_addr = 28'b0;
            end
            WRITE_BACK:
            begin
                mem_read = 1'b0;
                mem_write= 1'b1;
                mem_addr = {cache_tag,proc_index};
            end
            READ_FROM_MEM:
            begin
                mem_read = 1'b1;
                mem_write= 1'b0;
                mem_addr = {proc_tag,proc_index};
            end
            default:
            begin
                mem_read = 1'b0;
                mem_write= 1'b0;
                mem_addr = 28'b0;
            end
        endcase
    end

    always@(*)begin
        for(i = 0;i<8; i=i+1)begin
            blocks_next[i] = blocks[i];
        end
        proc_stall = 1'b0;
        case(state)
            IDLE:
            begin
                if(proc_read && ~proc_write)begin               // read
                    if(hit) begin
                        proc_stall = 1'b0;
                        state_next = IDLE;
                    end else begin
                        proc_stall = 1'b1;
                        if(valid) state_next = READ_FROM_MEM;
                        else      state_next = WRITE_BACK;
                    end
                end else if(~proc_read && proc_write)begin      // write
                    if(hit)begin
                        proc_stall = 1'b0;
                        state_next = IDLE;
                        blocks_next[proc_index][153] = 1'b0;    // set valid to faulse
                        if     (offset == 2'b00)blocks_next[proc_index][31:0] = proc_wdata;
                        else if(offset == 2'b01)blocks_next[proc_index][63:32] = proc_wdata;
                        else if(offset == 2'b10)blocks_next[proc_index][95:64] = proc_wdata;
                        else                    blocks_next[proc_index][127:96] = proc_wdata;
                    end else begin
                        proc_stall = 1'b1;
                        if(valid) state_next = READ_FROM_MEM;
                        else      state_next = WRITE_BACK;
                    end
                end else state_next = IDLE;
            end
            WRITE_BACK:
            begin
                proc_stall = 1'b1;
                if(~mem_ready) state_next = WRITE_BACK;
                else           state_next = READ_FROM_MEM;
            end
            READ_FROM_MEM:
            begin
                proc_stall = 1'b1;
                if(~mem_ready) state_next = READ_FROM_MEM;
                else begin
                    blocks_next[proc_index] = {1'b1,proc_tag,mem_rdata};
                    state_next = IDLE;
                end
            end
            default:
            begin
                state_next = IDLE;
            end
        endcase
    end
    
//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset ) begin
    if( proc_reset ) begin
        state <= 2'b00;
        for(i = 0;i<8;i=i+1)begin
            blocks[i] <= {1'b1,{153{1'b1}}};
        end 
    end
    else begin
        state <= state_next;
        for (i = 0; i<8; i=i+1)begin
            blocks[i] <= blocks_next[i];
        end
    end
end

endmodule

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
	
	input clk, rst_n;
	output ICACHE_ren, ICACHE_wen;
	output [29:0] ICACHE_addr;
	output [31:0] ICACHE_wdata;	//*******************??????????????????
	input  ICACHE_stall;
	input  [31:0] ICACHE_rdata;
	
	output DCACHE_ren, DCACHE_wen;
	output [29:0] DCACHE_addr;
	output [31:0] DCACHE_wdata;
	input  DCACHE_stall;
	input  [31:0] DCACHE_rdata;
	
	//(4)ALU
	wire Zero;
	
	//stall signal
	wire mem_stall;
	wire stall;
	
	
	//wire and reg declaration
		//(1)PC
	wire [31:0] PC4;
	wire [31:0] PC_w;
	reg  [31:0] PC_r;
		//IFID layer
	wire [31:0] IFID_PC4_w;
	wire [31:0] IFID_instruction_w;
	reg  [31:0] IFID_PC4_r;
	reg  [31:0] IFID_instruction_r;
		//(2)Control unit
	
	wire  [5:0] instruction;
	wire  [4:0] if_id_rs_r,if_id_rt_r,id_ex_rt_r; 
	wire  [5:0] func_ctrl;

	wire	jump_out;			//id		1:jal or j															  0:else
	wire	jr_pcaddr_out;		//id		1:jr or jalr, PC should jump to addr stored in register 0:else
	wire	branch_out;			//id		1:branch taken 													  0:else
	wire	flush_out;			//id		1:flush, branch or any kind of  J							  0:else

	wire    [1:0]aluop_out;		//ex     00:Itype 01:beq	10:Rtype	11:Jtype or	default
	wire	alusrc_out;			//ex		1:alu take instruction as operand 							  0:alu take data from register as operand
	wire	regwrite_ex_out;	//ex		1:regwrite instruction is in EX, tell forwarding unit   0:else
	wire	stall_for_lw_out;	/*ex		1:lw is in EX, PC & IFID should stay, 
												  set all three id_ex_xx_w to 0,								  0:else */
	wire	jal_addr_ex_out;  //ex		1:the data is tobe stored in $31								  0:else
	wire	jal_data_ex_out;
												  
	wire	memread_out;		//mem		1:lw is in MEM, tell forwarding unit		 				  0:else
	wire	memwrite_out;		//mem		1:write mem									 						  0:else
	wire	regwrite_mem_out;	//mem		1:regwrite instruction is in MEM, tell forwarding unit  0:else
	wire	jal_addr_mem_out; //mem		1:the data is tobe stored in $31								  0:else
	wire  jal_data_mem_out;

	wire	memtoreg_out;		//wb		1:reg_write_data=mem												  0:reg_write_data=alu
	wire	regwrite_wb_out;	//wb		1:regwrite instruction is in WB, tell forwarding unit   0:reg_not_write
	wire	jal_addr_out;		//wb		1:jal, store in $31												  0: if jalr: regdst_out=1 due to Rtype else: regdst decides...
	wire	jal_data_out;		//wb		1:jal or jalr, write the current PC into register		  0:else
	wire	regdst_out;  		//wb		1:reg_write_addr=instruction[15:11] 						  0:reg_write_addr=instruction[20:16]

	reg [2:0] id_ex_ex_r,id_ex_ex_w;		//id_ex
	reg [1:0]id_ex_m_r,id_ex_m_w;
	reg [4:0]id_ex_wb_r,id_ex_wb_w;

	reg [1:0]ex_mem_m_r,ex_mem_m_w;		//ex_mem
	reg [4:0]ex_mem_wb_r,ex_mem_wb_w;

	reg [4:0]mem_wb_wb_r,mem_wb_wb_w;	//mem_wb
	
		//(3)Register forwarding
	reg	[1:0] ForwardC,ForwardD;
	wire [4:0] wb_addr_alu_true;
	wire [4:0] wb_addr_mem_true;
	wire [4:0] wb_addr_wb_true;
		//(4)Register
	wire [5:0] IFID_inst_31_26;
	wire [31:0] IFID_Rs_data;
	wire [31:0] IFID_Rt_data;
	wire [15:0] IFID_inst_15_0;
	wire [4:0] IFID_Rs_addr;
	wire [4:0] IFID_Rt_addr;
	wire [4:0] IFID_Rd_addr;
	reg [31:0] register_w[31:0];
	reg [31:0] register_r[31:0];
	wire [31:0] ReadData_Rs, ReadData_Rt;
	integer i ;
	wire [4:0] Reg_Write_addr;
	wire [31:0] Reg_Write_data;
	wire [31:0] inst_sign_extend;
	wire [31:0] beq_addr;
	wire [31:0] jump_addr;
	wire [31:0] jr_addr;
		//IDEX
	wire [31:0] IDEX_PC4_w;
	wire [5:0]  IDEX_inst31_26_w;
	wire [31:0] IDEX_Rs_data_w;
	wire [31:0] IDEX_Rt_data_w;
	wire [15:0] IDEX_inst15_0_w;
	wire [4:0]  IDEX_Rt_addr_w;
	wire [4:0]  IDEX_Rd_addr_w;
	
	reg [31:0] IDEX_PC4_r;
	reg [5:0]  IDEX_inst31_26_r;
	reg [31:0] IDEX_Rs_data_r;
	reg [31:0] IDEX_Rt_data_r;
	reg [15:0] IDEX_inst15_0_r;
	reg [4:0]  IDEX_Rt_addr_r;
	reg [4:0]  IDEX_Rd_addr_r;
		//ALU	
	wire  [31:0] ALUin1;
	reg   [31:0] ALUin2;
	wire  [31:0] ALUadd;
	wire  [31:0] ALUsub;
	wire  [31:0] ALUand;
	wire  [31:0] ALUor;
	wire  [31:0] ALUxor;
	
	wire  [31:0] data2_forward;
	
	wire [31:0] addr_in_zero;
	wire [31:0] addr_in_sign;
	wire [4:0] shamt;
	wire [5:0] func;
	reg [31:0] ALUout;
	wire [4:0] IDEX_R;
		//EXMEM layer
	wire [31:0] EXMEM_data_w;	//aluout or PC4, JUDGE BY jal_data_ex_out
	wire [31:0] EXMEM_Rt_data_w;
	wire [4:0] EXMEM_R_w;
	
	reg  [31:0] EXMEM_data_r;
	reg  [31:0] EXMEM_Rt_data_r;
	reg  [4:0] EXMEM_R_r;
		//MEMWB layer
	wire [31:0] MEMWB_data_w;
	wire [4:0]  MEMWB_R_w;
	
	reg  [31:0] MEMWB_data_r;
	reg  [4:0]  MEMWB_R_r;
		//dummy
		wire [31:0] EXMEM_memout;
		
	
	// ICACHE
	assign ICACHE_ren=1'b1;
	assign ICACHE_wen=1'b0;
	assign ICACHE_wdata=32'd0;
	
	//stall
	assign mem_stall = ICACHE_stall | DCACHE_stall;
	
	//PC
	assign PC4 = PC_r+32'd4;
	assign ICACHE_addr = PC_r[31:2];
	assign jump_addr = {IFID_PC4_w[31:28], IFID_instruction_w[25:0],2'b0 };
	assign jump_out = (IFID_instruction_w[31:26]==6'b000010 || IFID_instruction_w[31:26]==6'b000011);
	assign PC_w = (stall) ? PC_r :
		   (jump_out==1'b1 && jr_pcaddr_out==1'b0 && branch_out==1'b0) ? jump_addr : 
		   (jump_out==1'b0 && jr_pcaddr_out==1'b1 && branch_out==1'b0) ? jr_addr :
		   (jump_out==1'b0 && jr_pcaddr_out==1'b0 && branch_out==1'b1) ? beq_addr : PC4;
		   
	
	
	//IFID
	assign IFID_instruction_w = (stall) ? IFID_instruction_r : 
								(flush_out) ? 32'd0 : ICACHE_rdata;
	assign IFID_PC4_w = (stall) ? IFID_PC4_r :
						(flush_out) ? 32'd0 : PC4;
	
	//Control unit
		//comb circuit
	assign  instruction = IFID_instruction_r[31:26];
	assign  if_id_rs_r = IFID_instruction_r[25:21];
	assign  if_id_rt_r = IFID_instruction_r[20:16];
	assign  id_ex_rt_r = IDEX_Rt_addr_r;
	assign  func_ctrl = IFID_instruction_r[5:0];
	//assign	jump_out=(instruction==6'b000010 || instruction==6'b000011);							//id
	assign	jr_pcaddr_out=(instruction==6'b000000)&&(func_ctrl==6'b001000 || func_ctrl==6'b001001);		//id
	assign	branch_out=(instruction==6'b000100)&&(Zero);									//id
	assign	flush_out=   ((instruction==6'b000000)&&(func_ctrl==6'b001000 || func_ctrl==6'b001001))||
						 ((instruction==6'b000100)&&(Zero));									//id

	assign aluop_out=id_ex_ex_r[2:1];																								//ex 
	assign alusrc_out=id_ex_ex_r[0];																									//ex
	assign regwrite_ex_out=id_ex_wb_r[3];																							//ex
	assign stall_for_lw_out=id_ex_m_r[1] &&(if_id_rs_r==id_ex_rt_r || if_id_rt_r==id_ex_rt_r)&& (id_ex_rt_r!=5'b00000);//ex 
	assign jal_addr_ex_out=id_ex_wb_r[2];																							//ex
	assign jal_data_ex_out=id_ex_wb_r[1];
								
	assign memread_out=ex_mem_m_r[1];			//mem
	assign memwrite_out=	ex_mem_m_r[0];			//mem
	assign regwrite_mem_out=ex_mem_wb_r[3]; 	//mem
	assign jal_addr_mem_out=ex_mem_wb_r[2];	//mem
	assign jal_data_mem_out=ex_mem_wb_r[1];
	assign memtoreg_out=ex_mem_wb_r[4];		//mem

	assign regwrite_wb_out=mem_wb_wb_r[3];		//wb
	assign jal_addr_out=mem_wb_wb_r[2];			//wb
	assign jal_data_out=mem_wb_wb_r[1];			//wb 
	assign regdst_out=id_ex_wb_r[0];  			//wb

	always@(*)begin
		if (mem_stall)begin																				//mem_stall
			id_ex_ex_w=id_ex_ex_r;
			id_ex_m_w=id_ex_m_r;
			id_ex_wb_w=id_ex_wb_r;
			ex_mem_m_w=ex_mem_m_r;
			ex_mem_wb_w=ex_mem_wb_r;
			mem_wb_wb_w=mem_wb_wb_r;

		end
		else if (id_ex_m_r[1] &&(if_id_rs_r==id_ex_rt_r || if_id_rt_r==id_ex_rt_r)&& (id_ex_rt_r!=5'b00000))begin 		//lw_stall
			id_ex_ex_w=3'b110;
			id_ex_m_w=2'b00;
			id_ex_wb_w=5'b00000;
			ex_mem_m_w=id_ex_m_r;
			ex_mem_wb_w=id_ex_wb_r;
			mem_wb_wb_w=ex_mem_wb_r;

		end
		else begin
			if(instruction==6'b000000)begin						//Rtype
				id_ex_ex_w=3'b100;
				id_ex_m_w=2'b00;
				id_ex_wb_w=(func_ctrl==6'b001001)?5'b01011:5'b01001;		//when jalr, jal_data_out set to 1

			end
			else if(instruction==6'b001000 || 					//addi
					instruction==6'b001100 || 					//andi
					instruction==6'b001101 || 					//ori
					instruction==6'b001110 ||					//xori
					instruction==6'b001010)begin				//slti
				id_ex_ex_w=3'b001;
				id_ex_m_w=2'b00;
				id_ex_wb_w=5'b01000;

			end
			else if(instruction==6'b100011)begin				//LW
				id_ex_ex_w=3'b001;
				id_ex_m_w=2'b10;
				id_ex_wb_w=5'b11000;

			end
			else if(instruction==6'b101011)begin				//SW
				id_ex_ex_w=3'b001;
				id_ex_m_w=2'b01;
				id_ex_wb_w=5'b00000;

			end
			else if(instruction==6'b000100)begin				//beq
				id_ex_ex_w=3'b010;
				id_ex_m_w=2'b00;
				id_ex_wb_w=5'b00000;

			end
			else if(instruction==6'b000010)begin				//J
				id_ex_ex_w=3'b110;
				id_ex_m_w=2'b00;
				id_ex_wb_w=5'b00000;

			end
			else if(instruction==6'b000011)begin				//JAL
				id_ex_ex_w=3'b110;
				id_ex_m_w=2'b00;
				id_ex_wb_w=5'b01110;

			end
			else begin			//break down
				id_ex_ex_w=3'b110;
				id_ex_m_w=2'b00;
				id_ex_wb_w=5'b00000;

			end
			ex_mem_m_w=id_ex_m_r;
			ex_mem_wb_w=id_ex_wb_r;
			mem_wb_wb_w=ex_mem_wb_r;
		end
	end
	
	// DCACHE
	assign DCACHE_ren=memread_out;
	assign DCACHE_wen=memwrite_out;
	assign DCACHE_addr=EXMEM_data_r[31:2];
	assign DCACHE_wdata=EXMEM_Rt_data_r;
	
	// stall
	
	assign stall = mem_stall | stall_for_lw_out;
	
	//Register forwarding

	assign wb_addr_alu_true=jal_addr_ex_out?5'b11111:IDEX_R;
	assign wb_addr_mem_true=jal_addr_mem_out?5'b11111:EXMEM_R_r;
	assign wb_addr_wb_true=jal_addr_out?5'b11111:MEMWB_R_r;

	always@(*)begin
		if(wb_addr_alu_true==IFID_Rs_addr && regwrite_ex_out && wb_addr_alu_true!=5'b00000)begin
			ForwardC=2'b01;
		end
		else if(wb_addr_mem_true==IFID_Rs_addr && regwrite_mem_out && wb_addr_mem_true!=5'b00000)begin
			ForwardC=2'b10;
		end
		else if(wb_addr_wb_true==IFID_Rs_addr && regwrite_wb_out && wb_addr_wb_true!=5'b00000)begin
			ForwardC=2'b11;
		end
		else begin
			ForwardC=2'b00;
		end
	end
	always@(*)begin
		if(wb_addr_alu_true==IFID_Rt_addr && regwrite_ex_out && wb_addr_alu_true!=5'b00000)begin
			ForwardD=2'b01;
		end
		else if(wb_addr_mem_true==IFID_Rt_addr && regwrite_mem_out && wb_addr_mem_true!=5'b00000)begin
			ForwardD=2'b10;
		end
		else if(wb_addr_wb_true==IFID_Rt_addr && regwrite_wb_out && wb_addr_wb_true!=5'b00000)begin
			ForwardD=2'b11;
		end
		else begin
			ForwardD=2'b00;
		end
	end
	
	//Register
	assign IFID_inst_31_26 = IFID_instruction_r[31:26];
	assign IFID_inst_15_0 = IFID_instruction_r[15:0];
	
	assign Reg_Write_addr = (jal_addr_out) ? 5'd31 : MEMWB_R_r;
	assign Reg_Write_data = MEMWB_data_r;
	
	assign ReadData_Rs = register_r[IFID_instruction_r[25:21]];
	assign ReadData_Rt = register_r[IFID_instruction_r[20:16]];
	assign IFID_Rs_data = (ForwardC==2'b01) ? EXMEM_data_w : 
						  (ForwardC==2'b10) ? MEMWB_data_w :
						  (ForwardC==2'b11) ? MEMWB_data_r : ReadData_Rs;
	assign IFID_Rt_data = (ForwardD==2'b01) ? EXMEM_data_w :
						  (ForwardD==2'b10) ? MEMWB_data_w :
						  (ForwardD==2'b11) ? MEMWB_data_r : ReadData_Rt;			  
	assign Zero = (IFID_Rs_data==IFID_Rt_data) ? 1'b1 : 1'b0;
	assign IFID_Rs_addr = IFID_instruction_r[25:21];
	assign IFID_Rt_addr = IFID_instruction_r[20:16];
	assign IFID_Rd_addr = IFID_instruction_r[15:11];
	assign inst_sign_extend = {{16{IFID_instruction_r[15]}}, IFID_inst_15_0};
	assign beq_addr = (inst_sign_extend << 2) + IFID_PC4_r;
	always @(*) begin
		register_w[0]=32'd0;
		for (i=1; i<32; i=i+1) begin
			register_w[i] = register_r[i];
		end
	
		if(regwrite_wb_out && Reg_Write_addr!=5'b00000) begin
			register_w[Reg_Write_addr] = Reg_Write_data;
		end
		else ;
		
	end
	
	//IDEX layer
	assign jr_addr = IFID_Rs_data;
	assign IDEX_PC4_w = (mem_stall) ? IDEX_PC4_r : IFID_PC4_r;
	assign IDEX_inst31_26_w = (mem_stall) ? IDEX_inst31_26_r : IFID_inst_31_26;
	assign IDEX_Rs_data_w = (mem_stall) ? IDEX_Rs_data_r : IFID_Rs_data;
	assign IDEX_Rt_data_w = (mem_stall) ? IDEX_Rt_data_r : IFID_Rt_data;
	assign IDEX_inst15_0_w = (mem_stall) ? IDEX_inst15_0_r : IFID_inst_15_0;
	assign IDEX_Rt_addr_w = (mem_stall) ? IDEX_Rt_addr_r : IFID_Rt_addr;
	assign IDEX_Rd_addr_w = (mem_stall) ? IDEX_Rd_addr_r : IFID_Rd_addr;
	
	//ALU	
	assign addr_in_zero = {16'b0, IDEX_inst15_0_r};
	assign addr_in_sign = {{16{IDEX_inst15_0_r[15]}}, IDEX_inst15_0_r};
	
	
	assign shamt = IDEX_inst15_0_r[10:6];
	assign func = IDEX_inst15_0_r[5:0];
	
	
	assign IDEX_R = (regdst_out) ? IDEX_Rd_addr_r : IDEX_Rt_addr_r ;

	assign ALUin1=IDEX_Rs_data_r;
	assign data2_forward=IDEX_Rt_data_r;
	always@(*)begin
		case(aluop_out)
		2'b00: begin	//I-type
			if((IDEX_inst31_26_r==6'b001100)||(IDEX_inst31_26_r==6'b001101)||(IDEX_inst31_26_r==6'b001110)) begin	//addi
				ALUin2 = addr_in_zero;
			end
			else begin	//slti
				ALUin2 = addr_in_sign;
			end
		end
		2'b10: begin	//R-type
			if((func==6'b100000)||(func==6'b100010)||(func==6'b100100)||(func==6'b100101)||(func==6'b100110)||(func==6'b101010))	begin	//add
				ALUin2 = data2_forward;
			end
			else begin	//sub
				ALUin2 = data2_forward;
			end
		end
		default : begin 
				ALUin2 = 32'd0;	
		end
		endcase	
	end
	assign	ALUadd = ALUin1 + ALUin2;
	assign	ALUsub = ALUin1 - ALUin2;
	assign	ALUand = ALUin1 & ALUin2;
	assign	ALUor  = ALUin1 | ALUin2;
	assign	ALUxor = ALUin1 ^ ALUin2;
	always@(*) begin
		case(aluop_out)
		2'b00: begin	//I-type
			if(IDEX_inst31_26_r==6'b001000) begin	//addi
				ALUout = ALUadd;
				end
			else if(IDEX_inst31_26_r==6'b001100) begin	//andi
				ALUout = ALUand;
				end
			else if(IDEX_inst31_26_r==6'b001101) begin	//ori
				ALUout = ALUor;
				end
			else if(IDEX_inst31_26_r==6'b001110) begin	//xori
				ALUout = ALUxor;
				end
			else if(IDEX_inst31_26_r==6'b001010) begin	//slti
				ALUout = ALUsub[31] ? 32'd1 : 32'd0;
				end
			else if(IDEX_inst31_26_r==6'b100011) begin	//lw
				ALUout = ALUadd;
				end
			else if(IDEX_inst31_26_r==6'b101011) begin	//sw
				ALUout = ALUadd;
				end
			else begin
				ALUout = 32'd0;	//do nothing
				end
			end
		2'b10: begin	//R-type
			if(func==6'b100000)	begin	//add
				ALUout = ALUadd;
				end
			else if(func==6'b100010) begin	//sub
				ALUout = ALUsub;
				end
			else if(func==6'b100100) begin	//and
				ALUout = ALUand;
				end
			else if(func==6'b100101) begin	//or
				ALUout = ALUor;
				end
			else if(func==6'b100110) begin	//xor
				ALUout = ALUxor;
				end
			else if(func==6'b100111) begin	//nor
				ALUout = ~ALUor;
				end
			else if(func==6'b000000) begin	//sll
				ALUout = ALUin1 << shamt;		//no influence to NOP
				end
			else if(func==6'b000011) begin	//sra
				ALUout = ALUin1 >>> shamt;
				end
			else if(func==6'b00010)	begin	//srl
				ALUout = ALUin1 >> shamt;
				end
			else if(func==6'b101010) begin	//slt
				ALUout = ALUsub[31] ? 32'd1 : 32'd0;
				end
			else if(func==6'b001000) begin	//jr
				ALUout = 32'd0;
				end
			else if(func==6'b001001) begin	//jalr
				ALUout = 32'd0;
				end
			else begin
				ALUout = 32'd0;
				end
			end
		default : begin 
				ALUout = 32'd0; 	
			end
		endcase	
	end
	
	//EXMEM layer
	assign EXMEM_data_w = (mem_stall) ? EXMEM_data_r : 
						  (jal_data_ex_out) ? IDEX_PC4_r : ALUout;
	assign EXMEM_Rt_data_w = (mem_stall) ? EXMEM_Rt_data_r : IDEX_Rt_data_r;
	assign EXMEM_R_w = (mem_stall) ? EXMEM_R_r : IDEX_R;
	
	//MEMWB layer
	assign EXMEM_memout=DCACHE_rdata;
	
	assign MEMWB_data_w = (mem_stall) ? MEMWB_data_r : 
						  (memtoreg_out) ? EXMEM_memout : EXMEM_data_r;
	
	assign MEMWB_R_w = (mem_stall) ? MEMWB_R_r : EXMEM_R_r;
	
	
	//sequential circuit
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			//PC
			PC_r <=32'd0;
			
			//control unit
			id_ex_ex_r<=3'b110;
			id_ex_m_r<=2'b00;
			id_ex_wb_r<=5'b00000;
			ex_mem_m_r<=2'b00;
			ex_mem_wb_r<=5'b00000;
			mem_wb_wb_r<=5'b00000;
			
			//Register
			for (i=0; i<32; i=i+1) begin   
			register_r[i]<=32'd0;
			end
			
			//IFID 
			IFID_PC4_r <= 32'd0;
			IFID_instruction_r <= 32'd0;
			
			//IDEX
			IDEX_PC4_r <= 32'd0;
			IDEX_inst31_26_r <= 6'd0;
			IDEX_Rs_data_r <= 32'd0;
			IDEX_Rt_data_r <= 32'd0;
			IDEX_inst15_0_r <= 16'd0;
			IDEX_Rt_addr_r <= 5'd0;
			IDEX_Rd_addr_r <= 5'd0;
			
			//EXMEM
			EXMEM_data_r <= 32'd0;
			EXMEM_Rt_data_r <= 32'd0;
			EXMEM_R_r <= 5'd0;
			
			//MEMWB
			MEMWB_data_r <= 32'd0;
			MEMWB_R_r <= 5'd0;
		end
		else begin
			//PC
			PC_r <= PC_w;
			
			//CONTROL UNIT
			id_ex_ex_r<=id_ex_ex_w;
			id_ex_m_r<=id_ex_m_w;
			id_ex_wb_r<=id_ex_wb_w;
			ex_mem_m_r<=ex_mem_m_w;
			ex_mem_wb_r<=ex_mem_wb_w;
			mem_wb_wb_r<=mem_wb_wb_w;
			
			//REGISTER
			for (i=0; i<32; i=i+1) begin   
				register_r[i]<=register_w[i];
			end
			
			//IFID 
			IFID_PC4_r <= IFID_PC4_w;
			IFID_instruction_r <= IFID_instruction_w;
			
			//IDEX
			IDEX_PC4_r <= IDEX_PC4_w;
			IDEX_inst31_26_r <= IDEX_inst31_26_w;
			IDEX_Rs_data_r <= IDEX_Rs_data_w;
			IDEX_Rt_data_r <= IDEX_Rt_data_w;
			IDEX_inst15_0_r <= IDEX_inst15_0_w;
			IDEX_Rt_addr_r <= IDEX_Rt_addr_w;
			IDEX_Rd_addr_r <= IDEX_Rd_addr_w;
			
			//EXMEM
			EXMEM_data_r <= EXMEM_data_w;
			EXMEM_Rt_data_r <= EXMEM_Rt_data_w;
			EXMEM_R_r <= EXMEM_R_w;
			
			//MEMWB
			MEMWB_data_r <= MEMWB_data_w;
			MEMWB_R_r <= MEMWB_R_w;
		end
	end
endmodule

