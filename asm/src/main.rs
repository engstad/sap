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
    Br11,
    Br8,
    R8,
    R8X,
    RR4,
    LS2,
    CALC,
    SPEC,
}

fn cls_info(cls: Class) -> (u8, u8, u8, u8, u8) {
    match cls {
        //            Rs  Code    CZ OZ IZ
        Class::Br11 => (0, 0b00001, 5, 0, 11),
        Class::Br8  => (0, 0b001__, 3, 5, 8),
        Class::R8   => (1, 0b01___, 2, 3, 8),
        Class::R8X  => (1, 0b00010, 5, 0, 8),
        Class::RR4  => (2, 0b100__, 3, 3, 4),
        Class::LS2  => (3, 0b1011_, 4, 2, 2),
        Class::CALC => (3, 0b110__, 3, 4, 0), // 3 + 4 + 3*3 = 16
        Class::SPEC => (2, 0b1111_, 4, 4, 8),
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Opc {
    INCI,
    DECI,
    MOVI,
    LDRI,
    SHLI,
    ASRI,
    LSRI,
    RORI,
    ADDI,
    SUBI,
    ADD,
    SUB,
    BRA,
    OUT,
    HALT,
    NOP,
}

fn info(opc: Opc) -> (Class, u8) {
    match opc {
        Opc::BRA => (Class::Br11, 0b0),
        //
        Opc::INCI => (Class::R8, 0b000),
        Opc::DECI => (Class::R8, 0b001),
        // CMPI
        Opc::MOVI => (Class::R8, 0b011),
        Opc::LDRI => (Class::R8X, 0),
        // LSP
        // SSP
        // ISP
        // DSP
        Opc::SHLI => (Class::RR4, 0b000),
        Opc::LSRI => (Class::RR4, 0b001),
        Opc::ASRI => (Class::RR4, 0b010),
        Opc::RORI => (Class::RR4, 0b011),
        Opc::ADDI => (Class::RR4, 0b100),
        // ?
        Opc::SUBI => (Class::RR4, 0b110),
        // ?
        Opc::ADD => (Class::CALC, 0b0000),
        Opc::SUB => (Class::CALC, 0b0001),

        Opc::NOP => (Class::SPEC, 0b0000),
        Opc::HALT => (Class::SPEC, 0b1111),
        Opc::OUT => (Class::SPEC, 0b1110),
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
    Dh(Vec<u16>)
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
    character::complete::{
        alpha1, char, digit1, multispace0, /*multispace1, one_of*/
        space0,
    },
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
    map(alpha1, |s: &str| {
        Oper::Str(s.to_string())
    })(i)
}

fn parse_opc<'a>(i: &'a str) -> IResult<&'a str, Opc, VerboseError<&'a str>> {
    alt((
        map(tag("bra"), |_| Opc::BRA),
        map(tag("inci"), |_| Opc::INCI),
        map(tag("deci"), |_| Opc::DECI),
        map(tag("movi"), |_| Opc::MOVI),
        map(tag("ldri"), |_| Opc::LDRI),
        map(tag("shli"), |_| Opc::SHLI),
        map(tag("asri"), |_| Opc::ASRI),
        map(tag("lsri"), |_| Opc::LSRI),
        map(tag("rori"), |_| Opc::RORI),
        map(tag("addi"), |_| Opc::ADDI),
        map(tag("subi"), |_| Opc::SUBI),
        map(tag("add"), |_| Opc::ADD),
        map(tag("sub"), |_| Opc::SUB),
        map(tag("out"), |_| Opc::OUT),
        map(tag("nop"), |_| Opc::NOP),
        map(tag("halt"), |_| Opc::HALT),
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
    map(tuple((parse_opc, preceded(space0, parse_opers))),
        |(opc, opers)| Stmt::Inst(opc, opers))(input)
}

fn parse_label(input: &str) -> Res<&str, Stmt> {
    map(tuple((alpha1, char(':'))),
        |s: (&str, char)| Stmt::Label(s.0.to_string()))(input)
}

fn parse_stmt(input: &str) -> Res<&str, Stmt> {
    context(
        "opers", alt((parse_inst, parse_label))
    )(input)
}

fn parse_program(input: &str) -> Res<&str, Vec<Stmt>> {
    context(
        "program",
        many0(map(tuple((multispace0, parse_stmt)), |(_, s)| s)),
    )(input)
}

const DEBUG: bool = false;

fn encode_stmt(stmt: &Stmt, _pc: u16, labels: &HashMap<String, u16>) -> Result<Option<u16>, String> {
    match stmt {
        Stmt::Org(_) => Err("not implemented".to_string()),
        Stmt::Label(_) => {
            Ok(None)
        },
        Stmt::Data(_) => Err("not implemented".to_string()),
        Stmt::Inst(opcode, opers) => {
            if DEBUG { println!("opcode: {:?}", opcode); }

            let (cls, opc) = info(*opcode);
            if DEBUG { println!("cls: {:?}, opc: {:?}", cls, opc); }
            let (regs, cls_code, code_sz, opc_sz, imm_sz) = cls_info(cls);
            if DEBUG {
                println!("regs: {}, cls_code: {}, code_sz: {}, opc_sz: {}, imm_sz: {} [{}]",
                         regs, cls_code, code_sz, opc_sz, imm_sz, code_sz + opc_sz + 3 * regs + imm_sz);
            }
            assert!(opc < 1 << opc_sz);

            if DEBUG { println!("opers: {:?}", opers); }

            let mut res =
                (cls_code as u16) << (16 - code_sz) | (opc as u16) << (16 - code_sz - opc_sz);

            if cls == Class::Br11 {
                if opers.len() != 1 { return Err("Too many operands to BRA".into()); }
                if let Oper::Str(s) = &opers[0] {
                    if let Some(dst) = labels.get(s) {
                        res |= dst; // :TODO: Relative
                        return Ok(Some(res));
                    } else {
                        return Err(format!("Couldn't find label {}", s));
                    }
                } else {
                    return Err("Invalid operand to BRA".into());
                }
            } else if cls != Class::SPEC {
                assert!(regs + if imm_sz > 0 { 1 } else { 0 } == opers.len() as u8);

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
            },
            Stmt::Inst(_, _) => {
                pc += 2;
            },
            _ => {}
        }
    }

    pc = 0;
    for stmt in prog.iter() {
        let r = encode_stmt(stmt, pc, &labels)?;
        if let Some(r) = r {
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
    bra end"###,
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
                Stmt::Inst(Opc::BRA, vec![Oper::Str("end".into())])
            ]
        ))
    );

    if let Ok((_, stmts)) = res {
        let enc = encode_program(&stmts);
        assert_eq!(enc, Ok(vec![0b100_100_001_010_0011,
                                0b1111_0000__0000_0000,
                                0b00000__000_0000_0010,
        ]));
    }
}

fn main() {
    let prog_src = include_str!("fib.asm");
    let prog_stmts = parse_program(prog_src);
    let prog_encoding = match prog_stmts {
        Ok(stmts) =>
            match encode_program(&stmts.1) {
                Ok(e) => e,
                Err(e) => { println!("Error: {}", e); return; }
            },
        Err(e) => {
            println!("could not encode: {}", e);
            return
        }
    };

    for i in 0..256/2 {
        if i < prog_encoding.len() {
            let p1 = prog_encoding[i];
            print!("{:02x} {:02x}", p1 & 0xFF, (p1 >> 8));
            println!();
        } else {
            println!("FF FF");
        }
    }
}
