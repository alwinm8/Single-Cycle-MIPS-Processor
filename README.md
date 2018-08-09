# Single Cycle MIPS Processor
## The Processor
– The goal for this project was to create a single cycle MIPS processor using Verilog HDL that can perform a subset of MIPS instructions including pseudo instructions. Throughout the semester, we implemented various modules found in the datapath, and for the final project, we are required to connect the entire CPU datapath and see if it can produce the correct output for a given data segment and a text segment. The instructor modified the datapath for the subset of instruction required for this implementation. Since there are separate data and text segments, the program counter must start at 0x3000, where the text segment starts. The instructions that must be supported are Arithmetic/logical: `add, and, div, mul, mult, or, sll, slt, sra, srl, sub`; Constant manipulation: `lui`; Control transfer: `beq, bne, j, jal, jr`; Memory reference: `lw, sw. Other: mfhi, mflo`. Mul is a pseudo instruction different than mult.
## Modules
The modules that I have implemented in my solution are:
- [x] **CPU** - A module to emulate the entire single cycle processor.
- [x] **Control** - A module that takes an opcode from an instruction and sets the control outputs needed for the specific instruction. 
- [x] **ALUControl** - A module that takes an ALUop code from the control unit and sets the ALU Control to perform the needed operation.
- [x] **DMU** - A module that emulates the RAM, or the data memory unit, which holds all the stored data.
- [x] **Registers** - A module to emulate the register array for read and write.
- [x] **ALU** - A module that forms arithmetic logic unit that performs bitwise operations and outputs the result of the computation.
- [x] **SignExtend** - A module that extends a 16 bit input to 32 bits
- [x] **PC** - A module that sends the current address to instruction memory and signal the Adder.
- [x] **Adder** - A module that increments address by 4 and return to the PC.
- [x] **IM** - A module that simulates instruction memory, prints current address.
- [x] **Clock** - A module which periodically changes the output signal.
- [x] **MUX** - A module that is a 2-to-1 MUX that takes two 32 bits inputs and a select bit to select which input to use.
- [x] **MUX5** - A module that is a 2-to-1 MUX that takes two 5 bit inputs and a select bit to select which input to use.
- [x] **SyscallControl** - A module that is called during a syscall operation and executes a function based on $v0.
