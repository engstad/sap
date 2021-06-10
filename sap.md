# Introduction

Ben Eater's famous series brought light to the SAP (Simple As Possible) architecture, often used to
introduce students to basic micro-electronics and programming.

There are several problems with the original instruction set:

    * The memory is only 16 bytes.
    * The instruction set is very limited.
    * There are very few registers (accumulator, program counter, address and status register).

This works perfectly well for a simple-as-possible architecture, but the student is left to wonder what
are the next steps? In search of that, we aim to create a RISC-like 16-bit architecture, which we shall
call SAP16.

# The SAP16 architecture.

It is important to ask, why not an 8-bit architecture? The problem with this is that an 8-bit addressing
space can cover only 256 bytes, which is quite small. During the 8-bit era, multiple remedies were invented
to circumvent this limitation. The 6502 processor used "banking", essentially using two registers to address
memory. Other architectures used 16-bit "address" registers on top of 8-bit general-purpose registers.

In order to reduce complexity, we will simply use all 16-bit registers. This frees up quite a bit of space
for the instruction register, so that we can add in most all normal instructions. We will also have 16
of these registers, though the first 8 will be more general purpose than the last. If we were to be symmetric,
we would spend 12 bits just for an `ADD D, A, B` instruction (executing `d = a + b;`), and since we can easily
make up 16 different operations, we have essentially used up all the instruction opcode space already. However,
with 8 registers, we only use 9 bits, thus allowing space for many other instructions.

Now, many instructions need less than 3 registers. For instance `INC D, A` (performing `d += a;`), or `LDR D, A`
(performing `d = *a;`). On the other hand, jumps and branches needs offsets or addresses, nescessitating encoding
values ("immediates") into the instruction. A common solution to this problem is to introduce instruction *classes*,
so that decoding the instructions remain fairly easy.

## Instruction Classes



# Classes

```
  0000_ciii_iiii_iiii = Branch11 [0000_]
  0001_cDDD_iiii_iiii = LdStImm8 [0001_]
  001c_cccc_iiii_iiii = CBranch8 [001__]
  01cc_cDDD_iiii_iiii = RegImm8N [01___]
  100c_ccDD_DAAA_iiii = Reg2Imm4 [100__]
  1011_cccc_DDDA_AAii = LdStImm2 [1011_]
  110c_cccD_DDAA_ABBB = Reg3Imm0 [110__]
  111x_xxxx_xxxx_xxxx = Special1 [111__]
```

## BRANCH

* BRA/BAL opc5, imm11          [00000, 00001] // 0
* Bxx     opc4, cnd4, imm8     [0010, 0011]   // 2,3

## REG + IMM8

* INC/DEC opc5, dst3, imm8     [01000, 01001] // 4
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

# Instructions

Rd = dest
Rn = src0
Rm = src1

* Adjust SP
* Sign/zero-extend: SXB, UXB.
* CBZ, CBN (compare and branch)
* Push/Pop registers
* Reverse Bytes (REV16)
* MLA, MUL, SMLAL (signed madd long), SMULL (signed mul long), UMLAL (unsigned madd long), UMULL (unsigned mul long)
* SDIV, UDIV
* Ops on high registers: ADD, CMP, MOV. (Two variants: D: `OPC Rd, Rm*` or N: `OPC Rm*, Rn`.)
* STR, STB, LDR, LDUB, LDSB,
* Wrapping and saturating ADD/SUB.
* Various sign-extend and ADDs.
* SIMD: Multiple
* CLZ, QADD, QDADD, QSUB, RBIT, REV, SEL.
*

# PIPELINING

## Stage 1 (Instruction Fetch)

Loads the instruction at the IP.

## Stage 2 (Instruction Decode)

Converts instruction into control signals.
Will set the next PC for branches?

## Stage 3 (Execution 1)

## Stage 4 (Execution 2)
