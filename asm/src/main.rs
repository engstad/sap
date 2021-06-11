/*

Assembly language has 3 types of instruction statements (stmt):

 - Instructions
   - Opcode mnemonic (sometimes extended ones)
   - Operands
 - Data definitions
 - Assembly directives
   - Labels
   - Macros
   - Directives
   - Comments
*/

use nom::*;
use std::collections::HashMap;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Class {
    Branch12,
    CBranch5,
    Reg1Imm8,
    Reg2Imm4,
    LdStImm2,
    Reg3Imm0,
    Special0,
}

fn cls_info(cls: Class) -> (u8, u8, u8, u8, u8) {
    match cls {
        //                 Rs  Code     CZ OZ IZ
        Class::Special0 => (2, 0b0000_, 4, 4, 8),
        Class::Reg3Imm0 => (3, 0b00___, 2, 5, 0),  // 30 of 32

        Class::LdStImm2 => (3, 0b0100_, 4, 2, 2),
        Class::Reg1Imm8 => (1, 0b01___, 2, 3, 8),  //  7 of 8

        Class::Reg2Imm4 => (2, 0b10___, 2, 4, 4),

        Class::Branch12 => (0, 0b1100_, 4, 0, 12),
        Class::CBranch5 => (2, 0b11___, 2, 3, 5),   // 6 of 8
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Opc {

    SLT,   SLTU,   Res0,  Res1,   DIV,   DIVU,   REM,   REMU,
    SLL,   SRL,    SRA,   ROR,    AND,   OR,     XOR,   BIC,
    ADD,   SUB,    MUL,   MULH,   MULHU, MULHSU, MULB,  Res2,

    LR,    SR,     // LdStImm2
    LRR,   SRR,    // Reg1Imm8
    INCI,  DECI,
    MOVI,  LUI,

    LBU,   XORI,   ORI,   ANDI,  // Reg2Imm4
    SLLI,  SRLI,   SRAI,  RORI,
    ADDI,  SLTI,   SUBI,  SLTIU, // Note: SLTIU rd, rs, 0 <=> SNEZ rd, rs
    LH,    LB,     STH,   STB,

    JMP,
    BEQ,   BNE,
    BLT,   BGE,
    BLTU,  BGEU,


    HALT,
    OUT, /*IN, JR, SXB, UXB, REV8, PAUSE,
    MOV,
    JALR,
    CLZ, RBIT, RBIT8, ZIP, BCDL, BCDH, FBCD, Res4,
    AUIPC,
    DIVB, DIVUB, REMB, REMUB, */

    NOP /* pseudo */
}

fn info(opc: Opc) -> (Class, u8) {
    match opc {
        Opc::SLT  => (Class::Reg3Imm0, 0b01000),
        Opc::SLTU => (Class::Reg3Imm0, 0b01001),
        // Free
        // Free
        Opc::DIV  => (Class::Reg3Imm0, 0b01100),
        Opc::DIVU => (Class::Reg3Imm0, 0b01101),
        Opc::REM  => (Class::Reg3Imm0, 0b01110),
        Opc::REMU => (Class::Reg3Imm0, 0b01111),
        Opc::SLL  => (Class::Reg3Imm0, 0b10000),
        Opc::SRL  => (Class::Reg3Imm0, 0b10001),
        Opc::SRA  => (Class::Reg3Imm0, 0b10010),
        Opc::ROR  => (Class::Reg3Imm0, 0b10011),
        Opc::AND  => (Class::Reg3Imm0, 0b10100),
        Opc::OR   => (Class::Reg3Imm0, 0b10101),
        Opc::XOR  => (Class::Reg3Imm0, 0b10110),
        Opc::BIC  => (Class::Reg3Imm0, 0b10111),

        Opc::ADD  => (Class::Reg3Imm0, 0b11000),
        Opc::SUB  => (Class::Reg3Imm0, 0b11001),
        Opc::MUL  => (Class::Reg3Imm0, 0b11010),
        Opc::MULH => (Class::Reg3Imm0, 0b11011),
        Opc::MULHU => (Class::Reg3Imm0, 0b11100),
        Opc::MULHSU  => (Class::Reg3Imm0, 0b11101),
        Opc::MULB  => (Class::Reg3Imm0, 0b11110),
        // Free
        Opc::LR => (Class::LdStImm2, 0),
        Opc::SR => (Class::LdStImm2, 1),
        Opc::LRR => (Class::Reg1Imm8, 0b010),
        Opc::SRR => (Class::Reg1Imm8, 0b011),
        Opc::INCI => (Class::Reg1Imm8, 0b100),
        Opc::DECI => (Class::Reg1Imm8, 0b101),
        Opc::MOVI => (Class::Reg1Imm8, 0b110),
        Opc::LUI => (Class::Reg1Imm8, 0b111),
        //
        Opc::LBU  => (Class::Reg2Imm4, 0b0000),
        Opc::XORI => (Class::Reg2Imm4, 0b0001),
        Opc::ORI  => (Class::Reg2Imm4, 0b0010),
        Opc::ANDI => (Class::Reg2Imm4, 0b0011),

        Opc::SLLI => (Class::Reg2Imm4, 0b0100),
        Opc::SRLI => (Class::Reg2Imm4, 0b0101),
        Opc::SRAI => (Class::Reg2Imm4, 0b0110),
        Opc::RORI => (Class::Reg2Imm4, 0b0111),

        Opc::ADDI => (Class::Reg2Imm4, 0b1000),
        Opc::SLTI => (Class::Reg2Imm4, 0b1001),
        Opc::SUBI => (Class::Reg2Imm4, 0b1010),
        Opc::SLTIU => (Class::Reg2Imm4, 0b1011),

        Opc::LH  => (Class::Reg2Imm4, 0b1100),
        Opc::LB  => (Class::Reg2Imm4, 0b1101),
        Opc::STH => (Class::Reg2Imm4, 0b1110),
        Opc::STB => (Class::Reg2Imm4, 0b1111),

        Opc::JMP => (Class::Branch12, 0),
        Opc::BEQ => (Class::CBranch5, 2),
        Opc::BNE => (Class::CBranch5, 3),
        Opc::BLT => (Class::CBranch5, 4),
        Opc::BGE => (Class::CBranch5, 5),
        Opc::BLTU => (Class::CBranch5, 6),
        Opc::BGEU => (Class::CBranch5, 7),
        // =====================================
        // =====================================
        Opc::HALT => (Class::Special0, 0b1111),
        Opc::OUT => (Class::Special0, 0b1110),
        // =====================================
        Opc::NOP => (Class::Special0, 0b1101),
        Opc::Res0 => (Class::Special0, 0b0000),
        Opc::Res1 => (Class::Special0, 0b0000),
        Opc::Res2 => (Class::Special0, 0b0000),
    }
}

type Res<T, U> = IResult<T, U, error::VerboseError<T>>;

#[derive(Debug, PartialEq, Clone)]
pub enum Oper {
    Num(i32),
    Reg(u8),
    Str(String),
}

#[derive(Debug, PartialEq, Clone)]
pub enum Data {
    Db(Vec<u8>),
    Dh(Vec<u16>),
}

#[derive(Debug, PartialEq, Clone)]
pub enum Stmt {
    Org(u16),
    Label(String),
    Inst(Opc, Vec<Oper>),
    Data(Data),
}

use nom::{
    branch::alt,
    bytes::complete::tag,
    character::complete::{alpha1, char, digit1, multispace0 /*multispace1, one_of*/, space0},
    combinator::{/*cut,*/ map, map_res, opt},
    error::{context, VerboseError},
    multi::many0,
    sequence::{/*delimited,*/ preceded, /*terminated,*/ tuple},
    IResult, /*Parser,*/
};

fn parse_num<'a>(i: &'a str) -> IResult<&'a str, Oper, VerboseError<&'a str>> {
    alt((
        map_res(digit1, |s: &str| s.parse::<i32>().map(Oper::Num)),
        map(preceded(tag("-"), digit1), |s: &str| {
            Oper::Num(-1 * s.parse::<i32>().unwrap())
        }),
    ))(i)
}

fn parse_reg<'a>(i: &'a str) -> IResult<&'a str, Oper, VerboseError<&'a str>> {
    map(preceded(tag("r"), digit1), |s: &str| {
        Oper::Reg(s.parse::<u8>().unwrap())
    })(i)
}

fn parse_str<'a>(i: &'a str) -> IResult<&'a str, Oper, VerboseError<&'a str>> {
    map(alpha1, |s: &str| Oper::Str(s.to_string()))(i)
}

fn parse_opc<'a>(i: &'a str) -> IResult<&'a str, Opc, VerboseError<&'a str>> {
    alt((
        alt((
            map(tag("xori"), |_| Opc::XORI),
            map(tag("xor"), |_| Opc::XOR),
            map(tag("subi"), |_| Opc::SUBI),
            map(tag("sub"), |_| Opc::SUB),
            map(tag("sth"), |_| Opc::STH),
            map(tag("stb"), |_| Opc::STB),
            map(tag("srr"), |_| Opc::SRR),
            map(tag("srli"), |_| Opc::SRLI),
            map(tag("srl"), |_| Opc::SRL),
            map(tag("srai"), |_| Opc::SRAI),
            map(tag("sra"), |_| Opc::SRA),
            map(tag("sr"), |_| Opc::SR),
            map(tag("slli"), |_| Opc::SLLI),
            map(tag("sll"), |_| Opc::SLL),
            map(tag("rori"), |_| Opc::RORI),
            map(tag("ror"), |_| Opc::ROR),
            map(tag("out"), |_| Opc::OUT),
            map(tag("ori"), |_| Opc::ORI),
            map(tag("or"), |_| Opc::OR),
        )),
        alt((
            map(tag("nop"), |_| Opc::NOP),
            map(tag("mulhu"), |_| Opc::MULHU),
            map(tag("mulhsu"), |_| Opc::MULHSU),
            map(tag("mulh"), |_| Opc::MULH),
            map(tag("mulb"), |_| Opc::MULB),
            map(tag("mul"), |_| Opc::MUL),
            map(tag("movi"), |_| Opc::MOVI),
            map(tag("lui"), |_| Opc::LUI),
            map(tag("lrr"), |_| Opc::LRR),
            map(tag("lr"), |_| Opc::LR),
            map(tag("lh"), |_| Opc::LH),
            map(tag("lbu"), |_| Opc::LBU),
            map(tag("lb"), |_| Opc::LB),
            map(tag("jmp"), |_| Opc::JMP),
            map(tag("inci"), |_| Opc::INCI),
            map(tag("halt"), |_| Opc::HALT),
            map(tag("deci"), |_| Opc::DECI),
            map(tag("bne"), |_| Opc::BEQ),
            map(tag("bltu"), |_| Opc::BEQ),
            map(tag("blt"), |_| Opc::BEQ),
        )),
        alt((
            map(tag("bic"), |_| Opc::BIC),
            map(tag("bgeu"), |_| Opc::BEQ),
            map(tag("bge"), |_| Opc::BEQ),
            map(tag("beq"), |_| Opc::BEQ),
            map(tag("andi"), |_| Opc::ANDI),
            map(tag("and"), |_| Opc::AND),
            map(tag("addi"), |_| Opc::ADDI),
            map(tag("add"), |_| Opc::ADD),
        )),
    ))(i)
}

fn parse_oper(input: &str) -> Res<&str, Oper> {
    alt((parse_reg, parse_num, parse_str))(input)
}

fn parse_opers(input: &str) -> Res<&str, Vec<Oper>> {
    many0(map(
        tuple((preceded(space0, parse_oper), opt(char(',')))),
        |(s, _)| s,
    ))(input)
}

fn parse_inst(input: &str) -> Res<&str, Stmt> {
    map(
        tuple((parse_opc, preceded(space0, parse_opers))),
        |(opc, opers)| Stmt::Inst(opc, opers),
    )(input)
}

fn parse_label(input: &str) -> Res<&str, Stmt> {
    map(tuple((alpha1, char(':'))), |s: (&str, char)| {
        Stmt::Label(s.0.to_string())
    })(input)
}

fn parse_stmt(input: &str) -> Res<&str, Stmt> {
    context("opers", alt((parse_inst, parse_label)))(input)
}

fn parse_program(input: &str) -> Res<&str, Vec<Stmt>> {
    context(
        "program",
        map(tuple((many0(map(tuple((multispace0, parse_stmt)), |(_, s)| s)),
                   multispace0)),
            |(r,_)| r)
    )(input)
}

const DEBUG: bool = false;

fn encode_stmt(
    stmt: &Stmt,
    _pc: u16,
    labels: &HashMap<String, u16>,
) -> Result<Option<u16>, String> {
    match stmt {
        Stmt::Org(_) => Err("not implemented".to_string()),
        Stmt::Label(_) => Ok(None),
        Stmt::Data(_) => Err("not implemented".to_string()),
        Stmt::Inst(opcode, opers) => {
            if DEBUG {
                println!("opcode: {:?}", opcode);
            }

            let (cls, opc) = info(*opcode);
            if DEBUG {
                println!("cls: {:?}, opc: {:?}", cls, opc);
            }
            let (regs, cls_code, code_sz, opc_sz, imm_sz) = cls_info(cls);
            if DEBUG {
                println!(
                    "regs: {}, cls_code: {}, code_sz: {}, opc_sz: {}, imm_sz: {} [{}]",
                    regs,
                    cls_code,
                    code_sz,
                    opc_sz,
                    imm_sz,
                    code_sz + opc_sz + 3 * regs + imm_sz
                );
            }
            assert!(opc < 1 << opc_sz);

            if DEBUG {
                println!("opers: {:?}", opers);
            }

            let mut res =
                (cls_code as u16) << (16 - code_sz) | (opc as u16) << (16 - code_sz - opc_sz);

            if cls == Class::Branch12 {
                if opers.len() != 1 {
                    return Err("Too many operands to JMP".into());
                }
                if let Oper::Str(s) = &opers[0] {
                    if let Some(dst) = labels.get(s) {
                        return Ok(Some(res | dst)); // :TODO: Relative
                    } else {
                        return Err(format!("Couldn't find label {}", s));
                    }
                } else if let Oper::Num(n) = &opers[0] {
                    return Ok(Some(res | *n as u16));
                } else {
                    return Err("Invalid operand to JMP".into());
                }
            } else if cls != Class::Special0 {
                let required = regs + if imm_sz > 0 { 1 } else { 0 };
                if opers.len() != required as usize {
                    return Err(format!("Wrong number of arguments ({}) given to {:?}, it needs {}.", opers.len(),
                                       opcode, required));
                }

                let mut pos = 3 * regs + imm_sz;
                for i in 0..opers.len() {
                    if (i as u8) < regs {
                        if let Oper::Reg(n) = opers[i] {
                            pos -= 3;
                            res |= (n as u16) << pos;
                        } else {
                            assert!(false);
                        }
                    } else if imm_sz > 0 {
                        if let Oper::Num(n) = opers[i] {
                            assert!(n < 1 << imm_sz);
                            res |= n as u16;
                        } else {
                            assert!(false);
                        }
                    }
                }
            } else {
                if *opcode == Opc::OUT {
                    if let Oper::Reg(n) = opers[0] {
                        res |= n as u16
                    }
                }
            }
            Ok(Some(res))
        }
    }
}

fn encode_program(prog: &[Stmt]) -> Result<Vec<u16>, String> {
    let mut res = vec![];

    let mut labels = HashMap::new();
    let mut pc = 0u16;

    for stmt in prog.iter() {
        match stmt {
            Stmt::Label(s) => {
                labels.insert(s.clone(), pc);
            }
            Stmt::Inst(_, _) => {
                pc += 2;
            }
            _ => {}
        }
    }

    pc = 0;
    for stmt in prog.iter() {
        let r = encode_stmt(stmt, pc, &labels)?;
        if let Some(r) = r {
            if DEBUG {
                println!("{:04x}", r);
            }
            res.push(r);
        }
    }
    Ok(res)
}

#[test]
fn test() {
    let res = parse_program(
        r###"
    addi r1, r2, 3
end:
    nop
    jal end"###,
    );

    println!("{:?}", res);
    assert_eq!(
        res,
        Ok((
            "",
            vec![
                Stmt::Inst(Opc::ADDI, vec![Oper::Reg(1), Oper::Reg(2), Oper::Num(3)]),
                Stmt::Label("end".into()),
                Stmt::Inst(Opc::NOP, vec![]),
                Stmt::Inst(Opc::JMP, vec![Oper::Str("end".into())])
            ]
        ))
    );

    if let Ok((_, stmts)) = res {
        let enc = encode_program(&stmts);
        assert_eq!(
            enc,
            Ok(vec![
                0b10_1000_001_010_0011,
                0b0000_1101__0000_0000,
                0b1100__0000_0000_0010,
            ])
        );
    }

    //assert!(false);
}

fn main() {
    let prog_src = include_str!("fib.asm");
    let prog_stmts = parse_program(prog_src);
    let prog_encoding = match prog_stmts {
        Ok(stmts) => {
            if stmts.0.len() > 0 {
                println!("Not parsed: `{}`", stmts.0);
                return;
            }
            match encode_program(&stmts.1) {
                Ok(e) => e,
                Err(e) => {
                    println!("Error: {}", e);
                    return;
                }
            }
        }
        Err(e) => {
            println!("could not encode: {}", e);
            return;
        }
    };

    for i in 0..256 / 2 {
        if i < prog_encoding.len() {
            let p1 = prog_encoding[i];
            print!("{:02x} {:02x} ", p1 & 0xFF, (p1 >> 8));
        } else {
            print!("FF FF ");
        }
        if i % 7 == 7 - 1 { println!(); }
    }
}
