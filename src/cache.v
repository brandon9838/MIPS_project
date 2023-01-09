module cache(
    clk,
    rst_n,

    cpu_read,
    cpu_write,
    cpu_addr,
    cpu_rdata,
	cpu_wdata,
	cpu_stall,
    
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== parameters ============================
    parameter IDLE          = 2'b00;
    parameter WRITE_BACK    = 2'b01;
    parameter READ_FROM_MEM = 2'b10;

//==== input/output ============================
    input	clk,rst_n;
	
    // processor interface
    input	cpu_read, cpu_write;
    input   [29:0] cpu_addr;
    output  reg	[31:0] cpu_rdata;
	input   [31:0] cpu_wdata;
	output	reg	cpu_stall;
    
	// memory interface
    output	reg	mem_read, mem_write;
	output  reg	[27:0] mem_addr;
	input	[127:0] mem_rdata;
    output	reg	[127:0] mem_wdata;
	input	mem_ready;
    
//==== wire/reg ================================
    reg		[1:0] state_w, state_r;
    reg 	[154:0] blocks_w[7:0], blocks_r[7:0];
	//cpu_tag = cpu_addr[29:5],  cpu_index = cpu_addr[4:2], cpu_block_offset = cpu_addr[1:0]
    //valid = block_match[154] , dirty= block_match[153], cache_tag = block_match[152:128],
    wire 	[24:0] cpu_tag;
    wire 	[2:0]  cpu_index;
    wire 	[1:0]  cpu_block_offset;
    wire 	[154:0] block_selected;
    wire 	valid, dirty, hit;
    wire 	[24:0] cache_tag;
    integer i;
    
//==== combinational circuit ==============================
    
	assign cpu_tag = cpu_addr[29:5];
    assign cpu_index = cpu_addr[4:2];
    assign cpu_block_offset = cpu_addr[1:0];
    assign block_selected = blocks_r[cpu_index];
    assign valid  = block_selected[154];
	assign dirty  = block_selected[153];
    assign hit = (cpu_tag == cache_tag) ? 1'b1: 1'b0;
    assign cache_tag = block_selected[152:128];
	
	always@(*)begin //State combinational circuit
		case(state_r)		
			IDLE:begin 
				if (cpu_read || cpu_write)begin
					if 		(hit && valid)		state_w=IDLE;	
					else if (!dirty)	state_w=READ_FROM_MEM;
					else 				state_w=WRITE_BACK;
				end
				else state_w=IDLE;
			end	

			WRITE_BACK:begin
				if (!mem_ready)	state_w=WRITE_BACK;
				else			state_w=READ_FROM_MEM;
			end

			READ_FROM_MEM:begin
				if (!mem_ready)	state_w=READ_FROM_MEM;
				else			state_w=IDLE;
			end
			
			default:begin
				state_w=IDLE;
			end
		endcase
	end
	
	always@(*)begin   // Cache block combinational circuit
		for(i = 0;i<8; i=i+1)begin	// default value
            blocks_w[i] = blocks_r[i];
        end
		
		if (cpu_write && state_r==IDLE && hit)begin	//on cpu write hit
			blocks_w[cpu_index][153]=1'b1;	//set dirty bit
			if     (cpu_block_offset == 2'b00)	blocks_w[cpu_index][31:0] = cpu_wdata;
            else if(cpu_block_offset == 2'b01)	blocks_w[cpu_index][63:32] = cpu_wdata;
            else if(cpu_block_offset == 2'b10)	blocks_w[cpu_index][95:64] = cpu_wdata;
            else                    			blocks_w[cpu_index][127:96] = cpu_wdata;
		end
		
		else if (state_r==READ_FROM_MEM && mem_ready)begin // on memory read
			blocks_w[cpu_index][154:153]=2'b10;	//set valid and !dirty
			blocks_w[cpu_index][152:128]=cpu_tag;
			blocks_w[cpu_index][127:0]	=mem_rdata;
		end
	end
	
	always@(*)begin //CPU combinational circuit
		cpu_rdata = (cpu_block_offset == 2'b00)? block_selected[31:0]:
					(cpu_block_offset == 2'b01)? block_selected[63:32]:
					(cpu_block_offset == 2'b10)? block_selected[95:64]: block_selected[127:96];
		
		if ((state_w==READ_FROM_MEM || state_w==WRITE_BACK) || 
			(state_r==READ_FROM_MEM || state_r==WRITE_BACK))begin
			cpu_stall=1;
		end
		else cpu_stall=0;
	end
	
    always@(*)begin   // Memory combinational circuit
		mem_wdata = block_selected[127:0];
        
		case(state_r)
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
                mem_addr = {cache_tag,cpu_index};
            end
            READ_FROM_MEM:
            begin
                mem_read = 1'b1;
                mem_write= 1'b0;
                mem_addr = {cpu_tag,cpu_index};
            end
            default:
            begin
                mem_read = 1'b0;
                mem_write= 1'b0;
                mem_addr = 28'b0;
            end
        endcase
    end
	
//==== sequential circuit =================================
	always@( posedge clk or negedge rst_n ) begin
		if(!rst_n) begin
			state_r <= IDLE;
			for(i = 0;i<8;i=i+1)begin
				blocks_r[i] <= 0;
			end 
		end
		else begin
			state_r <= state_w;
			for (i = 0; i<8; i=i+1)begin
				blocks_r[i] <= blocks_w[i];
			end
		end
	end

endmodule