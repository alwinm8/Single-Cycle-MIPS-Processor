/*
 *  File: TB_CPU.v
 *  Author:	Mr. Alwin Mathew
 *	Modules:
 *      	CPU 		- A module to emulate the entire single cycle processor.
 *			Control		- A module that takes an opcode from an instruction and sets the control outputs needed for the specific instruction.
 *			ALUControl	- A module that takes an ALUop code from the control unit and sets the ALU Control to perform the needed operation.
 * 			DMU			- A module that emulates the RAM, or the data memory unit, which holds all the stored data.
 *			Registers 	- A module to emulate the register array for read and write.
 *			ALU			- A module that forms arithmetic logic unit that performs bitwise operations and outputs the result of the computation.
 *			SignExtend	- A module that extends a 16 bit input to 32 bits, this processor is a 32bit MIPS processor.
 *			PC 			- A module that sends the current address to instruction memory and signal the Adder.
 *			Adder 		- A module that increments address by 4 and return to the PC.
 *			IM 			- A module that simulates instruction memory, prints current address.
 *			Clock 		- A module which periodically changes the output signal.
 *			MUX			- A module that is a 2-to-1 MUX that takes two 32 bits inputs and a select bit to select which input to use.
 *			MUX5        - A module that is a 2-to-1 MUX that takes two 5 bit inputs and a select bit to select which input to use. 
 *			SyscallControl - A module that is called during a syscall operation and executes a function based on $v0.
 */

 /*
 *  Module:	CPU		- A module to emulate the entire single cycle MIPS processor, includes all modules found above.
 *  Ports:
 *  Author:	Mr. Alwin Mathew
 */
 module CPU();
 	//All wires for the datapath
	wire clock;
	wire [31:0] address, inaddress;
	wire [31:0] ipp;
	wire [31:0] opm;
	wire [31:0] jumpaddress;
	wire JAL;
	wire BNE;
	wire regdst;
	wire jump;
	wire branch;
	wire memtoreg;
	wire memread;
	wire regwrite;
	wire memwrite;
	wire ALUsrc;
	wire Syscallop;
	wire [1:0] ALUop;
	wire [4:0] regdstresult;
	wire [4:0] writereg;
	wire [31:0] MUXmemtoreg;
	wire [31:0] writedataregister;
	wire [31:0] data1, data2;
	wire [31:0] output32;
	wire [31:0] output32shifted;
	wire [31:0] ALUresult;
	wire branchcheck;
	wire bnecheck;
	wire branching;
	wire [31:0] branchout;
	wire [3:0] ALUControl;
	wire shift;
	wire jr;
	wire [31:0] jumpout;
	wire [31:0] a;
	wire [31:0] b;
	wire zero;
	wire [31:0] ALUout;
	wire [31:0] readdata;

 	//Clock
	Clock clock1(.clock(clock));

	//Program Counter
	PC pc(.address(address), .inaddress(inaddress), .clock(clock));
 	
	//Adder
	Adder adder(.ipp(ipp), .address(address));
	
	//Instruction Memory
	IM im1(.opm(opm), .address(address), .clock(clock));

	//Shift first 26 bits left 2 of opm instruction with first 4 bits of output of adde
	assign jumpaddress = {ipp[31:28], ({2'b00, opm[25:0]}<<2)};

	//Control
	Control Control1(.opcode(opm[31:26]), .func(opm[5:0]), .regdst(regdst), .jump(jump), .branch(branch), .memtoreg(memtoreg), .memread(memread), .memwrite(memwrite), .ALUsrc(ALUsrc), .regwrite(regwrite), .ALUop(ALUop), .JAL(JAL), .BNE(BNE), .Syscallop(Syscallop));
	
	//Syscall
	SyscallControl SyscallControl1(.Syscallop(Syscallop));

	//MUX 5-bit for Regdst
	MUX5 MUX5regdst1(.input1(opm[20:16]), .input2(opm[15:11]), .select(regdst), .result(regdstresult));

	//MUX for JAL - 1
	MUX MUXJAL1(.input1(MUXmemtoreg), .input2(ipp), .select(JAL), .result(writedataregister));

	//MUX 5-bit for JAL - 2
	MUX5 MUX5JAL2(.input1(regdstresult), .input2(5'd31), .select(JAL), .result(writereg));

	//Registers
	Registers Registers1(.read1(opm[25:21]), .read2(opm[20:16]), .writereg(writereg), .writedataregister(writedataregister), .write(regwrite), .clock(clock), .data1(data1), .data2(data2));

	//Sign Extend
	SignExtend SignExtend1(.input16(opm[15:0]), .output32(output32));

	//Shift output of sign extend 2
	assign output32shifted = output32 << 2;

	//ALU Control Unit
	ALUControl ALUControl1(.ALUop(ALUop), .opcode(opm[31:26]), .func(opm[5:0]), .ALUControl(ALUControl), .shift(shift), .jr(jr));

	//MUX for ALUsrc
	MUX MUXALUsrc1(.input1(data2), .input2(output32), .select(ALUsrc), .result(b));

	//ALU
	ALU ALU1(.ALUcontrol(ALUControl), .a(data1), .b(b), .ALUout(ALUout), .opm(opm), .zero(zero), .shamt(opm[10:6]), .clock(clock));

	//Data Memory Unit 
	DMU DMU1(.readdata(readdata), .address(address), .writedata(data2), .memwrite(memwrite), .memread(memread), .clock(clock));

	//MUX for memtoreg
	MUX MUXmemtoreg1(.input1(ALUout), .input2(readdata), .select(memtoreg), .result(MUXmemtoreg));

	//ALU for adder and sign extend, ALUcontrol = 2 for add function only
	ALU ALU2(.ALUcontrol(4'd2), .a(output32shifted), .b(ipp), .ALUout(ALUresult), .clock(clock));

	//Result of bitwise AND for branch flag from control and zero from ALUControl
	assign branchcheck = (branch & zero);

	//Result of bitwise AND for branch not equal from control and NOT zero and from ALUControl
	assign bnecheck = (BNE & (~zero));
	//Result of bitwise OR for result of previous results
	assign branching = (branchcheck | bnecheck);

	//MUX for branch check
	MUX MUXbranch1(.input1(ipp), .input2(ALUresult), .select(branching), .result(branchout));

	//MUX for jump check
	MUX MUXjump1(.input1(branchout), .input2(jumpaddress), .select(jump), .result(jumpout));

	//MUX for shift
	MUX MUXshift1(.input1(data1), .input2(output32), .select(shift), .result(a));

	//MUX for jr
	MUX MUXjr1(.input1(jumpout), .input2(ALUout), .select(jr), .result(inaddress));

initial 
	begin
	//Start the processor! 
	$display("The processor has started!");
	//Display current address, instruction, and all current register values, since it doesn't actually output anything...
	$monitor("address:%b\ninstruction:%b\n$zero=%b\n$at=%b\n$v0=%b\n$v1=%b\n$a0=%b\n$a1=%b\n$a2=%b\n$a3=%b\n$t0=%b\n$t1=%b\n$t2=%b\n$t3=%b\n$t4=%b\n$t5=%b\n$t6=%b\n$t7=%b\n$s0=%b\n$s1=%b\n$s2=%b\n$s3=%b\n$s4=%b\n$s5=%b\n$s6=%b\n$s7=%b\n$t8=%b\n$t9=%b\n$k0=%b\n$k1=%b\n$gp=%b\n$sp=%b\n$fp=%b\n$ra=%b\n", address, opm, CPU.Registers1.registerarray[0], CPU.Registers1.registerarray[1], CPU.Registers1.registerarray[2], CPU.Registers1.registerarray[3], CPU.Registers1.registerarray[4], CPU.Registers1.registerarray[5], CPU.Registers1.registerarray[6], CPU.Registers1.registerarray[7], CPU.Registers1.registerarray[8], CPU.Registers1.registerarray[9], CPU.Registers1.registerarray[10], CPU.Registers1.registerarray[11], CPU.Registers1.registerarray[12], CPU.Registers1.registerarray[13], CPU.Registers1.registerarray[14], CPU.Registers1.registerarray[15], CPU.Registers1.registerarray[16], CPU.Registers1.registerarray[17], CPU.Registers1.registerarray[18], CPU.Registers1.registerarray[19], CPU.Registers1.registerarray[20], CPU.Registers1.registerarray[21], CPU.Registers1.registerarray[22], CPU.Registers1.registerarray[23], CPU.Registers1.registerarray[24], CPU.Registers1.registerarray[25], CPU.Registers1.registerarray[26], CPU.Registers1.registerarray[27], CPU.Registers1.registerarray[28], CPU.Registers1.registerarray[29], CPU.Registers1.registerarray[30], CPU.Registers1.registerarray[31]);
	#250
	$finish;
end

endmodule 

 /*
 *  Module:	MUX		- A module that is a 2-to-1 MUX that takes two 32 bits inputs and a select bit to select which input to use. 
 *  Author:	Author:	Mr. Alwin Mathew
 *	Ports:
 *		input1 - input wire - Input #1 32bits
 *  	input2 - input wire - Input #2 32bits
 *		select - input wire - selector bit
 *		result - output reg - Forwarded output of the MUX function given by the select bit
 */

module MUX(input1, input2, select, result);
	input [31:0] input1, input2;
	input select;
	output [31:0] result;

	//Always check on the change of an input
	assign result = (select) ? input2 : input1;

endmodule

 /*
 *  Module:	MUX5	- A module that is a 2-to-1 MUX that takes two 5 bits inputs and a select bit to select which input to use. 
 *  Author:	Author:	Mr. Alwin Mathew
 *	Ports:
 *		input1 - input wire - Input #1 5bits
 *  	input2 - input wire - Input #2 5bits
 *		select - input wire - selector bit
 *		result - output reg - Forwarded output of the MUX function given by the select bit
 */

module MUX5(input1, input2, select, result);
	input [4:0] input1, input2;
	input select;
	output [4:0] result;

	//Always check on the change of an input
	assign result = (select) ? input2 : input1;

endmodule

 /*
 *  Module:	Control		- A module that takes an opcode from an instruction and sets the control outputs needed for the specific instruction.
 *  Author:	Author:	Mr. Alwin Mathew
 *	Ports:
 *		opcode- input wire -Arithmetic/logical: add, and, div, mul, mult, or, sll, slt, sra, srl, sub; 
 *							Constant manipulation: lui; 
 *							Control transfer: beq, bne, j, jal, jr
 *							Memory reference: lw, sw
 *							Other: mfhi, mflo	
 *		func- input wire - Specific function code that is meant to differentiate the function of the operation, relating to ALU
 *		regdst - output reg - Output port that determines how the desination register is specified 
 *		jump - output reg - Output port that enables loading the jump target address into the PC
 *		branch - output reg - Output port that combined with a condition test boolean to enable loading the branch target address into the PC.
 *		memtoreg - output reg - Output port that determines where the value to be written comes from.
 *		memread - output reg - Output port that enables a memory read for load instructions.
 *		memwrite - output reg - Output port that enables a memory write for store instructions.
 *		ALUsrc - output reg - Output port that selects the second source operand for the ALU.
 *		regwrite - output reg - Output port that enables a write to one of the registers. 
 *		Syscallop - output reg - Output port that signals the syscall module for syscall operations.
 *		ALUop - output reg - Output port that either specifies ALU operation or specifies the operation to be done.
 *		JAL - output reg - Output port that is set on JAL instructions.
 *		BNE - output reg - Output port that is set on BNE instructions. 
 */
 
 module Control(opcode, func, regdst, jump, branch, memtoreg, memread, memwrite, ALUsrc, regwrite, ALUop, JAL, BNE, Syscallop);
 
 // bits 31-26 from the instruction 
 input [5:0] opcode;
 // defines an operation when the instruction is an R-type or special
 input [5:0] func;
 
 output reg regdst;
 output reg jump;
 output reg branch;
 output reg memtoreg;
 output reg memread;
 output reg memwrite;
 output reg ALUsrc;
 output reg regwrite;
 output wire Syscallop;
 output reg [1:0] ALUop;
 output reg JAL;
 output reg BNE;
 
`define ADD  6'b000000		//func code = 100000
`define AND  6'b000000		//func code = 100100
`define DIV  6'b000000		//func code = 011010
`define MUL  6'b011100		//func code = 000010
`define MULT 6'b000000		//func code = 011000
`define JR   6'b000000		//func code = 001000
`define OR   6'b000000		//func code = 100101 
`define SRA  6'b000000		//func code = 000011
`define SLL  6'b000000		//func code = 000000
`define SLT  6'b000000		//func code = 101010
`define SRL  6'b000000		//func code = 000010
`define SUB  6'b000000		//func code = 100010
`define SYSCALL 6'b000000 	//func code = 001100
`define MFHI 6'b000000		//func code = 010000
`define MFLO 6'b000000		//func code = 010010
`define LUI  6'b001111
`define BEQ  6'b000100
`define BNE  6'b000101
`define LW   6'b100011
`define SW   6'b101011
`define J    6'b000010
`define JAL  6'b000011


//set flag for a syscall operation 
 assign Syscallop = (opcode == `SYSCALL) && (func == 6'b001100);
 
 always @(*) begin
 	if (Syscallop)
	begin
		$display("System call.");
		JAL = 0;
		BNE = 0;
		regdst = 1;
		ALUsrc = 0;
		memtoreg = 0;
		regwrite = 0;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 0;
		ALUop = 2'b10;
	end
	if (opcode == `ADD || opcode == `AND || opcode == `DIV || opcode == `MULT || opcode == `JR || opcode == `OR || opcode == `SRA || opcode == `SLL || opcode == `SLT || opcode == `SRL || opcode == `SUB  || opcode == `MFHI || opcode == `MFLO)
	begin
	//these are R type instructions, they all have the same control output, but there are function codes associated with each instruction
		$display("R-type.");
		JAL = 0;
		BNE = 0;
		regdst = 1;
		ALUsrc = 0;
		memtoreg = 0;
		regwrite = 1;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 0;
		ALUop = 2'b10;
	end
	else if (opcode == `MUL)
	begin
	//multiply pseudo-instruction
		$display("MUL Multiply.",);
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 1;
		memtoreg = 0;
		regwrite = 1;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 0;
		ALUop = 2'b11;
	end
	else if (opcode == `LUI)
	begin
		$display("Load upper immediate");
	//load upper immediate instruction
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 1;
		memtoreg = 0;
		regwrite = 1;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 0;
		ALUop = 2'b11;
	end
	else if (opcode == `LW)
	begin
	//load word instruction
	$display("Load word.");
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 1;
		memtoreg = 1;
		regwrite = 1;
		memread = 1;
		memwrite = 0;
		branch = 0;
		jump = 0;
		ALUop = 2'b00;
	end
	else if (opcode == `SW)
	begin
	//store word instruction
	$display("Store word");
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 1;
		memtoreg = 0;
		regwrite = 0;
		memread = 0;
		memwrite = 1;
		branch = 0;
		jump = 0;
		ALUop = 2'b00;
	end
	else if (opcode == `BEQ)
	begin
	//branch on equal
	$display("Branch on equal.");
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 0;
		memtoreg = 0;
		regwrite = 0;
		memread = 0;
		memwrite = 0;
		branch = 1;
		jump = 0;
		ALUop = 2'b01;
	end
	else if (opcode == `BNE)
	begin
	//branch on not equal instruction
	$display("Branch on not equal.");
		BNE = (opcode == `BNE);
		JAL = 0;
		regdst = 0;
		ALUsrc = 0;
		memtoreg = 1;
		regwrite = 0;
		memread = 0;
		memwrite = 1;
		branch = 1;
		jump = 0;
		ALUop = 2'b01;
	end
	else if (opcode == `J)
	begin
	//jump instruction
		$display("Jump.");
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 0;
		memtoreg = 0;
		regwrite = 0;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 1;
		ALUop = 2'b00;
	end
	else if (opcode == `JAL)
	begin
	//jump and link instruction
	$display("Jump and link.");
		JAL = (opcode == `JAL); //set JAL flag
		BNE = 0;
		regdst = 1;
		ALUsrc = 0;
		memtoreg = 1;
		regwrite = 1;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 1;
		ALUop = 2'b00;
	end
	else
	begin
	//not an instruction understood...
	$display("Unknown instruction.");
		JAL = 0;
		BNE = 0;
		regdst = 0;
		ALUsrc = 0;
		memtoreg = 0;
		regwrite = 0;
		memread = 0;
		memwrite = 0;
		branch = 0;
		jump = 0;
		ALUop = 2'b00;
	end
 end
 endmodule

  /*
 *  Module:	SyscallControl		- A module that handles syscall operations; supports print int, print string, and exit.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		Syscallop 	- 	Input wire - Input wire that will signal if the operation is a syscall operation, 
 */
 module SyscallControl(Syscallop);
   input Syscallop;
   reg [7:0] printchar;
   reg [10:0] index;

   always @(posedge Syscallop)
   begin
   		case (CPU.Registers1.registerarray[2])
   			1:
   			begin
   				$display("%d", CPU.Registers1.registerarray[4]);//print integer in $a0
   			end
   			4:
   			begin
   				index = CPU.Registers1.registerarray[4][10:0]; //print string from address found at $a0 

   				if (index[1:0] == 2'b00)   //check the data memory for the string, assume opposite endian-ness
   					printchar = CPU.DMU1.mem[index[10:2]][7:0];
   				if (index[1:0] == 2'b01)
   					printchar = CPU.DMU1.mem[index[10:2]][15:8];
   				if (index[1:0] == 2'b10)
   					printchar = CPU.DMU1.mem[index[10:2]][23:16];
   				if (index[1:0] == 2'b11)
   					printchar = CPU.DMU1.mem[index[10:2]][31:24];

   				while (printchar != 0)
   				begin
   					$write("%c", printchar);
   					index++;
   					if (index[1:0] == 2'b00)  //check the data memory for the string, assume opposite endian-ness
   					printchar = CPU.DMU1.mem[index[10:2]][7:0];
   					if (index[1:0] == 2'b01)
   					printchar = CPU.DMU1.mem[index[10:2]][15:8];
   					if (index[1:0] == 2'b10)
   					printchar = CPU.DMU1.mem[index[10:2]][23:16];
   					if (index[1:0] == 2'b11)
   					printchar = CPU.DMU1.mem[index[10:2]][31:24];
   				end
   			end
   			10:
   			begin
   				$finish; //end the processor
   			end
   		endcase
   end
 endmodule

  /*
 *  Module:	ALUControl		- A module that takes an ALUop code from the control unit and sets the ALU Control to perform the needed operation.
 *  Author:	Author:	Mr. Alwin Mathew
 *	Ports:
 *		ALUop - input wire -Arithmetic/logical: add, and, div, mul, mult, or, sll, slt, sra, srl, sub; 
 *							Constant manipulation: lui; 
 *							Control transfer: *beq, bne, j, jal, jr
 *							Memory reference: lw, sw
 *							Other: mfhi, mflo	
 *		func, opcode - input wire - Input ports that contain the function code and the opcode to determine what to set ALUControl
 *		shift 	   - output reg - Output reg which is set when a shift operation is being done.
 *		jr         - output reg - Output reg which is set when a jump register operation is being done.
 *	    ALUControl - output - Specific function code that is meant to differentiate the function of the operation, relating to ALU
 * 		Control lines	|	Function
 * 		----------------------------------------------
 * 		0000			|	AND
 * 		0001			|	OR
 * 		0010			|	add
 * 		0110			|	subtract
 * 		0111			|	set
 * 		1100			|	NOR
 * 		0011			|	MFHI
 * 		0100			|	SLL
 * 		0101			|	MFLO
 * 		1000			|	SRL
 * 		1001			|	SRA
 * 		1010			|	XOR
 * 		1011			|	SLTU
 * 		1101			|	MULT
 * 		1110			|	DIV
 * 		1111			|	LUI
 */
module ALUControl(ALUop, opcode, func, ALUControl, shift, jr);
	 input [1:0] ALUop;
	 input [5:0] func, opcode;
	 output reg [3:0] ALUControl;
	 output reg shift, jr;

	 initial
	 begin
	 //must set these to 0 initially because the MUXs will mess up the datapath if given a 'dont care x' value
	 	assign shift = 0;
	 	assign jr = 0;
	 end

	 always @(ALUop or func or opcode)
	 begin
	 	casex({ALUop,func})
	 		8'b10_001000:
	 		begin
	 			ALUControl=4'bxxxx; //jr
	 			jr = (func == 6'b001000); //set JR flag
	 		end
		 	8'b10_011010:ALUControl=4'b1110; //div
		 	8'b10_011000:ALUControl=4'b1101; //mult
		 	8'b11_000010:ALUControl=4'b1011; //mul
		 	8'b10_000000:
		 	begin
				ALUControl=4'b0100; //sll
				shift = (func == 6'b000000);
			end
		 	8'b10_000011:
		 	begin
		 		ALUControl=4'b1001; //sra
		 		shift = (func == 6'b000011);
		 	end
		 	8'b10_000010:
		 	begin
		 		ALUControl=4'b1000; //srl
		 		shift = (func == 6'b000010);
		 	end
		 	8'b10_010000:ALUControl=4'b0011; //mfhi
		 	8'b10_010010:ALUControl=4'b0101; //mflo
			8'b10_100000:ALUControl=4'b0010; //add
			8'b10_000010:ALUControl=4'b0110; //sub
			8'b10_100100:ALUControl=4'b0000; //and
			8'b10_100101:ALUControl=4'b0001; //or
			8'b10_101010:ALUControl=4'b0111; //slt
	 	endcase
	 	casex(opcode)
	 		6'b000101:ALUControl=4'b0110; //bne
	 		6'b000100:ALUControl=4'b0110; //beq
	 	    6'b100011:ALUControl=4'b0010; //lw
			6'b101011:ALUControl=4'b0010; //sw
	 		6'b001111:ALUControl=4'b1111; //lui 
	 		6'b000010:ALUControl=4'bxxxx; //j
	 		6'b000011:ALUControl=4'bxxxx; //jal
	 	endcase
	 end

endmodule


/*
 *  Module:	Registers 	- A module to emulate the register array for read and write.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		read1	-	Input port for the number of the first register to read.
 *		read2	-	Input port for the number of the second register to read.
 *		writereg-	Input port for the number of the register to write.
 *		writedataregister-  Input port for the data to write if write is set to true.
 *		write   -	Input bit to trigger write if it was set to true.
 *		clock 	- 	A single bit which toggles on a periodic basis.
 *		data1	-	Output port for the first register values read.
 *		data2	-	Output port for the second register value read.
 */
module Registers(read1, read2, writereg, writedataregister, write, clock, data1, data2);
	input [4:0] read1, read2, writereg;
	input [31:0] writedataregister;
	input write;
	input clock;
	output reg [31:0] data1, data2;
	//32 register array each with 32 bits
	reg [31:0] registerarray [31:0]; 

	initial
	begin
		//Register 0 is always 0
		registerarray[0] = 0;
	end
	//reads are enabled on negative edge clock cycles
	always @(negedge clock)
	begin
		data1 = registerarray[read1];
		data2 = registerarray[read2];
	end
	//writes are enabled on positive edge clock cycles
	always @(posedge clock)
	begin
		if (write)
		begin
			registerarray[writereg] = writedataregister;
			//Always assign zero
			registerarray[0] = 0;
		end
	end
endmodule

/*
 *  Module:	DMU			- A module that emulates the RAM, or the data memory unit, which holds all the stored data.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		clock -	A single bit which toggles on a periodic basis.
 *		memwrite - Input bit that if set to true will write writedata to the RAM.
 *		memread - Input bit that if set to true will read the RAM and write to readdata.
 *		address - Input wire that contains the address of the memory to be accessed.
 *		writedata- Input wire that contains the data to be written.
 *		readdata- Output wire that contains the data read from the RAM.
 */
module DMU(readdata, address, writedata, memwrite, memread, clock);
	input memwrite, memread, clock;
	input [31:0] address, writedata;
	output reg [31:0] readdata;
//32 bits of memory with 1024 entries for RAM
	reg [31:0] mem [0:1023];

	initial 
	begin
		$readmemb("fact-base=0x0000.dat", mem);
	end
//doesnt matter if positive edge or negative edge because memread and memwrite can't both be enabled
	always @(posedge clock)
	begin
		if (memread == 1)
			readdata = mem[address[11:2]];
	end

	always @(posedge clock)
	begin
		if (memwrite == 1)
			mem[address[11:2]] = writedata;
	end
endmodule

/*
 *  Module:	ALU			- A module that forms arithmetic logic unit that performs bitwise operations and outputs the result of the computation.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		ALUcontrol- Input port that is used to select the ALU operation to be performed.
 *		a         - Input port that contains the first input variable of the ALU operation.	
 *		b         - Input port that contains the second input variable of the ALU operation.
 *		ALUout	  - Output reg that contains the result of the ALU operation.
 *		zero	  - Output bit that is set to true if the result of the ALU operation is zero.
 *      hilo	  - Register that contains hi bits and low bits
 *		shamt	  - Input port that contains bits to shift for SLL and SRA functions
 */
module ALU(ALUcontrol, a, b, ALUout, opm, zero, shamt, clock);
	input clock;
	input signed [3:0] ALUcontrol;
	input signed [4:0] shamt;
	input signed [31:0] a, b;
	input [31:0] opm;
	output reg [31:0] ALUout;
	output reg zero;
	reg [63:0] hilo;

//set hi and lo to 0 initially
	initial 
	begin
		zero = 0;
		hilo = 0;
	end

//reevaluate if any of these changes, need to implement MUL
	always @(*)
	begin
		case(ALUcontrol)
			0: ALUout = a & b; //AND
			1: ALUout = a | b; //OR
			2: ALUout = a + b; //ADD
			3: ALUout = hilo[63:32]; //MFHI
			4: ALUout = b << shamt; //SLL
			5: ALUout = hilo[31:0]; //MFLO
			6: ALUout = a - b; //SUB
			7: ALUout = a < b ? 1 : 0; //SLT
			8: ALUout = b >> shamt; //SRL
			9: ALUout = b >> shamt; //SRA
			10: ALUout = a ^ b; //XOR
			12: ALUout = ~(a | b); //NOR
			13: hilo = a * b; //MULT
			14: 
			begin
				hilo[31:0] = a / b; //DIV
				hilo[63:32] = a % b;
			end
			15: ALUout = {opm[15:0],16'b0}; //LUI
			default: ALUout = 0;
		endcase
		//set zero to true if ALUout is zero
		zero = (ALUout == 0);
	end 
endmodule

/*
 *  Module:	SignExtend	- A module that extends a 16 bit input to 32 bits, this processor is a 32bit MIPS processor.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		input16	-	Input wire that contains the 16 bits to extend.		
 *		output32-	Output reg that contains the 32 bits of the extension of input16.
 */
module SignExtend(input16, output32);
	input [15:0] input16;
	output reg [31:0] output32;

	always @(input16)
	begin
		output32[31] = input16[15];
		output32[30] = input16[15];
		output32[29] = input16[15];
		output32[28] = input16[15];
		output32[27] = input16[15];
		output32[26] = input16[15];
		output32[25] = input16[15];
		output32[24] = input16[15];
		output32[23] = input16[15];
		output32[22] = input16[15];
		output32[21] = input16[15];
		output32[20] = input16[15];
		output32[19] = input16[15];
		output32[18] = input16[15];
		output32[17] = input16[15];
		output32[16] = input16[15];
		output32[15] = input16[15]; //this implmenetation takes the first 16 bits of the new output and places the last bit of the input into the first 16
		output32[14] = input16[14];
		output32[13] = input16[13];
		output32[12] = input16[12];
		output32[11] = input16[11];
		output32[10] = input16[10];
		output32[9] = input16[9];
		output32[8] = input16[8];
		output32[7] = input16[7];
		output32[6] = input16[6];
		output32[5] = input16[5];
		output32[4] = input16[4];
		output32[3] = input16[3];
		output32[2] = input16[2];
		output32[1] = input16[1];
		output32[0] = input16[0];
	end
endmodule
 
 
 /*
 *  Module:	PC		- A module that sends the current address to instruction memory and signal the Adder. If start is true, then address is set to true.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		address	-	Output address that is sent to instruction memory and adder.
 *		inaddress - Input port to receive the address.
 *		clock   -	Input port clock that is needed for a signal.
 *    start   - Input port to check if it has just been started
 */
module PC(address, inaddress, clock);
	input clock;
	reg start;
  //This input will set the PC to the input
	input [31:0] inaddress;
	output reg [31:0] address;

	initial
	begin
		start = 0;
		//program counter must start at 0x3000 because 0x0000 is the data segment and text should start at 0x3000
		address = 32'h3000;
	end

  always @(posedge clock)
  begin
  		if (start)
  		begin
  			address = inaddress;
  		end
  		if (!start)
  		begin
  		//0x3000 hex to start
  			address = 32'h3000;
      		start = 1;
      	end
   end
endmodule

 /*
 *  Module:	Adder		- A module that increments address by 4 and return to the PC.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		address	  -	Input port to receive the address.
 *		ipp -	Output address that is incremented by 4.
 */

module Adder(ipp, address);
	input [31:0] address;
	output [31:0] ipp;

  	//binary representation of the number 4
	parameter INCREMENT = 32'h00000004;

  	//increment ipp by INCREMENT and this output is sent out
	assign ipp = address + INCREMENT;

endmodule


 /*
 *  Module:	IM		- A module that simulates instruction memory, prints current address.
 *  Author:	Mr. Alwin Mathew
 *	Ports:
 *		opm 	- 	Output current instruction.
 *		address	-	Input address to check instruction memory.
 *		clock 	- 	Input clock that is needed for a signal.
 */
module IM(opm, address, clock);
	input [31:0] address;
	input clock;
	output [31:0] opm;
	//32 bits with 4k entries 
	reg [31:0] mem [0:4095];

	//read the instruction file and load it into the system, start at 3072 decimal because for some reason 0x3000 is that for an address bits 2->31...
	initial 
	begin
		$readmemb("fact-base=0x3000", mem, 3072);
	end

	assign opm = mem[address[31:2]];

endmodule

 /*
 *  Module:	Clock		- A module which periodically changes the output signal.
 *  Author:	Dr. Richard A. Goodrum, Ph.D.
 *	Ports:
 *		clock- O/P	wire	A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI.
 *	LO represents the number of ticks during which the output will be low.
 *  HI represents the number of ticks during which the output will be high.
 */
module Clock( clock );		// Establish the name of the module and ports.

	parameter LO = 10, HI = 10;
	output reg clock;			// Establish a storage location for the oscillating output signal.

	initial						// Loop structure which executes only once during start-up.
		clock = 0;				// clock is intended to start as a low value.

	always						// Loop structure which executes continuously after start-up.
		begin					// Only a code block within the loop.
			#LO	clock = ~clock;	// Toggle the value of clock after LO ticks.
			#HI	clock = ~clock;	// Toggle the value of clock after HI ticks.
		end						// Close the code block within the loop.

endmodule						// End of code for this module.
