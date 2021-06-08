SAP-1 instruction set is limited to 16, but with two registers, we can encode way more efficiently,
by looking at the instructions that are wasting bits.

    NOP: 16 instructions
    ADD: 16 instructions
    SUB: 16 instructions
    OUT: 16 instructions
    HLT: 16 instructions

Probably 4 out of 16 instructions: CTRL (TESTS), ALU1 (adds, subss), ALU2 (bit-logic), IO (NOP, HLT, OUT, IN).

    LDA, STA, LDB, STB, JMP, JC, JZ, JNZ : {opcode[3:0], addr[3:0]}
    ADDI, SUBI, LDI : { opcode[3:0], imm[3:0] }

Ideas: Have an instruction (TST/CMP) that checks for a condition, then a lot less branch and jump instructions.

We then have LD/STA and LD/STB. It is still not great to waste so much op-code space, and when the address space
increases (256 bytes => 8 bit addresses, 64 kB => 16 bit addresses), we would have to resort to loading immediates
using multiple bytes, e.g. (LDA, addr-byte) for 256 bytes and (LDA, HI-byte, LO-byte) for 64kB.

For 8-bit immediates, we could use the sequence (MHI, 4-bit) and (MLO, 4-bit), and then LDX A, X, LDX B, X, etc.
then we have reduced the amount of space needed for LDA, STA, LDB, STB, etc. This means that we can go to using
4 registers instead of just 1 or 2.

But we really want 16-byte addresses. We could do this (e.g. the Thumb architecture goes this way), but instead,
we can separate address registers from 8-bit general purpose registers.

Most branches are local, meaning that we could allow relative jumps. Reserving 00xx_xxxx for general instructions,
and 01xx_xxxx for immediate load, we can use 10xx_xxxx for

We can clearly not fit in 8 bits per instruction, so how about 16?

# Classes

```
  0000_xxx = BR11
  001x_xxx = BR8
  01xx_xxx = IMM8
  0001_xxx = IMM8
  100x_xxx = IMM4
  1011_xxx = LS-IM2
  110x_xxx = CALC
  111x_xxx = SPECIAL
```

## BRANCH

* BRA/BAL opc5, imm11          [00000, 00001] // 0
* Bxx     opc4, cnd4, imm8     [0010, 0011]   // 2,3

## REG + IMM8

* ADD/SUB opc5, dst3, imm8     [01000, 01001] // 4
* CMP/MOV opc5, dst3, imm8     [01010, 01011] // 5
* LSP/SSP opc5, dst3, imm8     [01100, 01101] // 6
* ASP/APC opc5, dst3, imm8     [01110, 01111] // 7

* LFL     opc5, dst3, imm8     [00010] // 1

## IMM4

* SHIFTS  opc6, dst3, src3, imm4 [1000_00, 1000_01, 1000_10, 1000_11] // 8
* ADD/SUB opc6, dst3, src3, imm4 [1001_0x, 1001_0x] // 9

## LOADS & STORES

* LDR/STR opc5, dst3, src3, src3, imm2 [10110, 10111] // B
  - LDR r0, r1, r2, imm4 (r0 = r1 + r2 << imm2),

## CALC

* regular opc7, dst3, src3, src3 [110xxxx] 16 opcodes // C,D
* special                                             // E,F


# PIPELINING

## Stage 1 (Instruction Fetch)

Loads the instruction at the IP.

## Stage 2 (Instruction Decode)

Converts instruction into control signals.
Will set the next PC for branches?

## Stage 3 (Execution 1)

## Stage 4 (Execution 2)
