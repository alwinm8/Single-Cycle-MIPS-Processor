/*
 *  File: tb_cpu.v
 *  Author:	Mr. Alwin Mathew
 *	Modules:
 *			tb_cpu		- A TestBench to demonstrate the functionality of register read, ALU, data memory unit, and sign extension unit. Takes input from file for data.
 *      Registers 		- A module to emulate the register array for read and write.
 *			DMU			- A module that emulates the RAM, or the data memory unit, which holds all the stored data.
 *			ALU			- A module that forms arithmetic logic unit that performs bitwise operations and outputs the result of the computation.
 *			SignExtend	- A module that extends a 16 bit input to 32 bits, this processor is a 32bit MIPS processor.
 *			Clock 		- A module which periodically changes the output signal.
 */

/*
 *  Module:	tb_cpu		- A TestBench to demonstrate the functionality of register read, ALU, data memory unit, and sign extension unit. Takes input from file for data.
 *  Author:	Author:	Mr. Alwin Mathew
 *	Ports:
 *		clock- O/P	wire	A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI.
 */
module tb_cpu();
//test ports for sign extend
	reg [15:0] input16;
	wire [31:0] output32;
//test ports for ALU
	reg signed [31:0] a, b;
	reg signed [3:0] ALUcontrol;
	wire signed [31:0] ALUout;
	wire zero;
//test ports for Registers
	reg [5:0] read1, read2, writereg;
	reg [31:0] writedataregister;
	reg write;
	wire clock;
	wire [31:0] data1, data2;
//test ports for DMU
	reg memwrite, memread;
	reg [31:0] address, writedata;
	wire [31:0] readdata;

//create instances of our modules for testing
	Clock Clock1(.clock(clock));
	SignExtend SE(.input16(input16), .output32(output32));
	ALU ArithmeticLU(.ALUcontrol(ALUcontrol), .a(a), .b(b), .ALUout(ALUout), .zero(zero));
	Registers Register(.read1(read1), .read2(read2), .writereg(writereg), .writedataregister(writedataregister), .write(write), .clock(clock), .data1(data1), .data2(data2));
	DMU DataMU(.readdata(readdata), .address(address), .writedata(writedata), .memwrite(memwrite), .memread(memread), .clock(clock));


//begin testing by assigning values 
	initial
	begin
//for sign extend testing, assign random values for input16 and check if the output is 32 bits with leading 0/1 for positive/negative
		$display("Sign Extend testing:");
		$monitor($time, "\tinput16=%16b, output32=%32b", input16, output32);
		#5	input16 = 0;
		#5	input16 = 10;
		#5	input16 = 170;
		#5	input16 = -20;
		#5	input16 = -220;
//for ALU testing, test each operation by assigning ALUcontrol the operation number and random numbers for evaluation
		$display("ALU testing:");
		$monitor($time, "\ta=%d, b=%d, ALUcontrol=%d, ALUout=%d, zero=%d", a, b, ALUcontrol, ALUout, zero);
		#5	a = 21; b = 3; ALUcontrol = 0;
		#5	a = 2; b = 5; ALUcontrol = 1;
		#5	a = 5; b = -4; ALUcontrol = 2;
		#5	a = 9; b = 10; ALUcontrol = 6;
		#5	a = 5; b = 5; ALUcontrol = 6;
		#5	a = 1; b = 3; ALUcontrol = 7;
		#5	a = 6; b = 5; ALUcontrol = 7;
		#5	a = 1; b = 3; ALUcontrol = 12;
//for registers testing, write 10 to all MIPS registers and check if it can be read 
		#5 writereg = 0; writedataregister = 10; write = 1;
	    #5 writereg = 1; 
	    #5 writereg = 2; 
	    #5 writereg = 3; 
	    #5 writereg = 4; 
	    #5 writereg = 5; 
	    #5 writereg = 6; 
	    #5 writereg = 7; 
	    #5 writereg = 8; 
	    #5 writereg = 9; 
	    #5 writereg = 10; 
	    #5 writereg = 11; 
	    #5 writereg = 12; 
	    #5 writereg = 13; 
	    #5 writereg = 14; 
	    #5 writereg = 15; 
	    #5 writereg = 16; 
	    #5 writereg = 17; 
	    #5 writereg = 18; 
	    #5 writereg = 19; 
	    #5 writereg = 20; 
	    #5 writereg = 21; 
	    #5 writereg = 22; 
	    #5 writereg = 23; 
	    #5 writereg = 24; 
	    #5 writereg = 25; 
	    #5 writereg = 26; 
	    #5 writereg = 27; 
	    #5 writereg = 28; 
	    #5 writereg = 29; 
	    #5 writereg = 30; 
	    #5 writereg = 31; 
	    #5 read1 = 0; read2 = 2;
	    #5;
	    $display("Registers testing:");
	    $display($time, "\tread1=%d, read2=%d, writereg=%d, writedataregister=%d,\n\t\t write=%d, data1=%d, data2=%d", read1, read2, writereg, writedataregister, write, data1, data2);
//for data memory unit, the RAM already contains random values that were assigned in the module, use random values for address and writedata and see if we can write to and read from the RAM
	    $display("Data Memory Unit testing (values in RAM are random, and address and writedata for testing):");
	    #5 memwrite=1; memread=0; address=$random; writedata=$random;
	    #5;
	    $display($time, "\tmemwrite=%d, memread=%d, address=%32b,\n\t\t writedata=%32b,\n\t\t readdata=%32b", memwrite, memread, address, writedata, readdata);
	    #5 memwrite=0; memread=1; address=$random; writedata=$random;
	    #5;
	    $display($time, "\tmemwrite=%d, memread=%d, address=%32b,\n\t\t writedata=%32b,\n\t\t readdata=%32b", memwrite, memread, address, writedata, readdata);
	    $finish;
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
	input [5:0] read1, read2, writereg;
	input [31:0] writedataregister;
	input write;
	input clock;
	output [31:0] data1, data2;
//32 register array each with 32 bits
	reg [31:0] registerarray [31:0]; 

	assign data1 = registerarray[read1];
	assign data2 = registerarray[read2];

	always
	begin
//write the register with the new value if write is set to true
		@(posedge clock) if (write) registerarray[writereg] <= writedataregister;
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
	integer i;

//for purposes of testing, fill RAM with random values, the random values assigned are only positive
	initial
	begin
		for (i = 0; i < 1024; i = i + 1)
		begin
			mem[i] = $random;
		end
	end


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
 */
module ALU(ALUcontrol, a, b, ALUout, zero);
	input signed [3:0] ALUcontrol;
	input signed [31:0] a, b;
	output reg signed [31:0] ALUout;
	output zero;
//set zero to true if ALUout is zero
	assign zero = (ALUout == 0);

//reevaluate if any of these changes
	always @(ALUcontrol, a, b)
	begin
		case(ALUcontrol)
			0: ALUout <= a & b;
			1: ALUout <= a | b;
			2: ALUout <= a + b;
			6: ALUout <= a - b;
			7: ALUout <= a < b ? 1 : 0;
			12: ALUout <= ~(a | b);
			default: ALUout <= 0;
		endcase
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
		output32[31] <= input16[15];
		output32[30] <= input16[15];
		output32[29] <= input16[15];
		output32[28] <= input16[15];
		output32[27] <= input16[15];
		output32[26] <= input16[15];
		output32[25] <= input16[15];
		output32[24] <= input16[15];
		output32[23] <= input16[15];
		output32[22] <= input16[15];
		output32[21] <= input16[15];
		output32[20] <= input16[15];
		output32[19] <= input16[15];
		output32[18] <= input16[15];
		output32[17] <= input16[15];
		output32[16] <= input16[15];
		output32[15] <= input16[15];
		output32[14] <= input16[14];
		output32[13] <= input16[13];
		output32[12] <= input16[12];
		output32[11] <= input16[11];
		output32[10] <= input16[10];
		output32[9] <= input16[9];
		output32[8] <= input16[8];
		output32[7] <= input16[7];
		output32[6] <= input16[6];
		output32[5] <= input16[5];
		output32[4] <= input16[4];
		output32[3] <= input16[3];
		output32[2] <= input16[2];
		output32[1] <= input16[1];
		output32[0] <= input16[0];
	end
endmodule

/*
 *  Module:	Clock		- A module which periodically changes the output signal.
 *  Author:	Dr. Richard A. Goodrum, Ph.D.
 *	Ports:
 *		clock- O/P	wire	A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI.
 */
module Clock( clock );		// Establish the name of the module and ports.
/*
 *	LO represents the number of ticks during which the output will be low.
 *  HI represents the number of ticks during which the output will be high.
 */
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


