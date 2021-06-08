use nom::*;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Cls {
    Br11, Br8, R8, R8X, RR4, LS2, CALC, SPEC
}

fn cls_info(cls: Cls) -> (u8, u8, u8, u8, u8) {
    match cls {
        //            Rs  Code    CZ  OZ IZ
        Cls::Br11 => (0, 0b00000, 5, 0, 11),
        Cls::Br8  => (0, 0b001__, 3, 5,  8),
        Cls::R8   => (1, 0b01___, 2, 3,  8),
        Cls::R8X  => (1, 0b00010, 5, 0,  8),
        Cls::RR4  => (2, 0b100__, 3, 3,  4),
        Cls::LS2  => (3, 0b1011_, 4, 2,  2),
        Cls::CALC => (3, 0b110__, 3, 4,  0), // 3 + 4 + 3*3 = 16
        Cls::SPEC => (2, 0b1111_, 4, 4,  8),
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Opc {
    MOVI,
    ADDI,
    OUT,
    HALT,
    NOP,
}

fn info(opc: Opc) -> (Cls, u8) {
    match opc {
        Opc::MOVI => (Cls::R8, 0b011),
        Opc::ADDI => (Cls::RR4, 0b100),

        Opc::NOP => (Cls::SPEC, 0b0000),
        Opc::HALT => (Cls::SPEC, 0b1111),
        Opc::OUT => (Cls::SPEC, 0b1110),
    }
}

const MOVI: u8 = 0b011;
const ADDI: u8 = 0b10;
const OUT: u8 = 0b1110;
const HALT: u8 = 0b1111;
const NOP: u8 = 0b0000;

fn encode_imm8(opc: u8, dst: u8, imm: u8) -> u16 {
    (0b01 as u16) << 14 | (opc as u16) << 11 | (dst as u16) << 8 | (imm as u16)
}

fn encode_imm4(opc: u8, dst: u8, src: u8, imm: u8) -> u16 {
    (0b100 as u16) << 13
        | (opc as u16) << 11
        | (dst as u16) << 7
        | (src as u16) << 4
        | ((imm & 0b1111) as u16)
}

fn encode_spec(opc: u8, src: u8) -> u16 {
    (0b1111 as u16) << 12 | (opc as u16) << 8 | (src as u16)
}

#[derive(Debug, PartialEq, Eq)]
struct Inst<'a> {
    items: Vec<&'a str>,
}

type Res<T, U> = IResult<T, U, error::VerboseError<T>>;

#[derive(Debug, PartialEq, Clone)]
pub enum Arg {
    Num(i32),
    Reg(u8),
    Str(String),
}

#[derive(Debug, PartialEq, Clone)]
pub enum Stmt {
    Lbl(String),
    Ins(Opc, Vec<Arg>),
}

use nom::{
    branch::alt,
    bytes::complete::tag,
    character::complete::{alpha1, char, digit1, multispace0 /*multispace1, one_of*/},
    combinator::{/*cut,*/ map, map_res, opt},
    error::{context, VerboseError},
    multi::many0,
    sequence::{/*delimited,*/ preceded, /*terminated,*/ tuple},
    IResult, /*Parser,*/
};

fn parse_num<'a>(i: &'a str) -> IResult<&'a str, Arg, VerboseError<&'a str>> {
    alt((
        map_res(digit1, |s: &str| {
            s.parse::<i32>().map(Arg::Num)
        }),
        map(preceded(tag("-"), digit1), |s: &str| {
            Arg::Num(-1 * s.parse::<i32>().unwrap())
        }),
    ))(i)
}

fn parse_reg<'a>(i: &'a str) -> IResult<&'a str, Arg, VerboseError<&'a str>> {
    map(preceded(tag("r"), digit1), |s: &str| {
        Arg::Reg(s.parse::<u8>().unwrap())
    })(i)
}

fn parse_str<'a>(i: &'a str) -> IResult<&'a str, Arg, VerboseError<&'a str>> {
    map(alpha1, |s: &str| Arg::Str(s.to_string()))(i)
}

fn parse_opc<'a>(i: &'a str) -> IResult<&'a str, Opc, VerboseError<&'a str>> {
    alt((
        map(tag("movi"), |_| Opc::MOVI),
        map(tag("addi"), |_| Opc::ADDI),
        map(tag("out"), |_| Opc::OUT),
        map(tag("nop"), |_| Opc::NOP),
        map(tag("halt"), |_| Opc::HALT),
    ))(i)
}

fn parse_stmt(input: &str) -> Res<&str, Stmt> {
    context(
        "args",
        map(
            tuple((
                parse_opc,
                preceded(
                    multispace0,
                    many0(map(
                        tuple((
                            preceded(multispace0, alt((parse_reg, parse_num))),
                            opt(char(',')),
                        )),
                        |(s, _)| s,
                    )),
                ),
            )),
            |(opc, args)| Stmt::Ins(opc, args),
        ),
    )(input)
}

fn encode_stmt(stmt: Stmt) -> Result<u16, String> {
    match stmt {
        Stmt::Lbl(_) => Err("not implemented".to_string()),
        Stmt::Ins(opcode, args) => {
            let (cls, opc) = info(opcode);
            println!("cls: {:?}, opc: {:?}", cls, opc);
            let (regs, cls_code, code_sz, opc_sz, imm_sz) = cls_info(cls);
            println!("regs: {}, cls_code: {}, code_sz: {}, opc_sz: {}, imm_sz: {} [{}]",
                     regs, cls_code, code_sz, opc_sz, imm_sz, code_sz + opc_sz + 3 * regs + imm_sz);

            assert!(opc < 1<<opc_sz);

            println!("args: {:?}", args);

            let mut res = (cls_code as u16) << (16 - code_sz) | (opc as u16) << (16 - code_sz - opc_sz);

            if cls != Cls::SPEC {
                assert!(regs + if imm_sz > 0 { 1 } else { 0 } == args.len() as u8);

                let mut pos = 3 * regs + imm_sz;
                for i in 0..args.len() {
                if (i as u8) < regs {
                    if let Arg::Reg(n) = args[i] {
                        pos -= 3;
                        res |= (n as u16) << pos;
                    } else {
                        assert!(false);
                    }
                } else if imm_sz > 0 {
                    if let Arg::Num(n) = args[i] {
                        assert!(n < 1<<imm_sz);
                        res |= n as u16;
                    } else {
                        assert!(false);
                    }
                }
                }
            } else {
                if opcode == Opc::OUT {
                    if let Arg::Reg(n) = args[0] {
                        res |= n as u16
                    }
                }
            }
            Ok(res)
        }
    }
}

fn encode_str(str: &str) -> Result<u16, String> {
    let parsed = match parse_stmt(str) {
        Ok(r) => r, Err(_e) => { return Result::Err("uh?".to_string()); }
    };
    let (_rest, res) = parsed;
    let tmp = encode_stmt(res);
    if let Ok(enc) = tmp {
        println!("{:04x} {:016b}", enc, enc);
    }
    tmp
}

#[test]
fn test() {
    let res = parse_stmt("addi r1, r2, 3\nnop");
    println!("{:?}", res);
    assert_eq!(
        res,
        Ok((
            "",
            Stmt::Ins(
                Opc::ADDI,
                vec![
                    Arg::Reg(1),
                    Arg::Reg(2),
                    Arg::Num(3)
                ]
            )
        ))
    );

    assert_eq!(encode_str("movi r1, 0"), Ok(0x5900));
    assert_eq!(encode_str("movi r2, 1"), Ok(0x5a01));
    assert_eq!(encode_str("addi r2, r1, 0"), Ok(0x9110));
    assert_eq!(encode_str("out r2"), Ok(0xfe02));
}

fn main() {
    let prog = [
        encode_imm8(MOVI, 1, 0),
        encode_imm8(MOVI, 2, 1),
        encode_imm4(ADDI, 2, 1, 0),
        encode_spec(OUT, 2),
        encode_imm4(ADDI, 1, 2, 0),
        encode_spec(OUT, 1),
        encode_imm4(ADDI, 2, 1, 0),
        encode_spec(OUT, 2),
        encode_imm4(ADDI, 1, 2, 0),
        encode_spec(OUT, 1),
        encode_imm4(ADDI, 2, 1, 0),
        encode_spec(OUT, 2),
        encode_imm4(ADDI, 1, 2, 0),
        encode_spec(OUT, 1),
        encode_imm4(ADDI, 2, 1, 0),
        encode_spec(OUT, 2),
        encode_imm4(ADDI, 1, 2, 0),
        encode_spec(OUT, 1),
        encode_spec(NOP, 0),
        encode_spec(HALT, 0),
    ];

    for p in prog.iter() {
        println!("{:02x} {:02x}", p & 0xFF, (p >> 8));
    }
}
