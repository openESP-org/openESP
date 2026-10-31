use winnow::{
    ModalResult, Parser,
    ascii::multispace0,
    combinator::{alt, delimited},
    token::{take_until, take_while},
};

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum Error {
    #[error("invalid calculator syntax at byte {0}")]
    Syntax(usize),
    #[error("unsupported calculator token {0}")]
    Unsupported(String),
    #[error("unbalanced calculator conditional")]
    Conditional,
    #[error("calculator stack underflow")]
    Stack,
    #[error("calculator value has the wrong type")]
    Type,
    #[error("non-finite calculator result")]
    Number,
    #[error("unavailable calculator variable {0}")]
    Variable(String),
    #[error("calculator case index or count is out of range")]
    Case,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Number(f64),
    Text(String),
}

impl Value {
    pub fn number(&self) -> Result<f64, Error> {
        match self {
            Self::Number(value) if value.is_finite() => Ok(*value),
            Self::Number(_) => Err(Error::Number),
            _ => Err(Error::Type),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Reference {
    pub category: char,
    pub name: String,
    pub unit: Option<String>,
}

impl Reference {
    pub fn new(category: char, name: &str, unit: Option<&str>) -> Self {
        Self {
            category: category.to_ascii_uppercase(),
            name: name.trim().to_ascii_uppercase(),
            unit: unit.map(|u| u.trim().to_ascii_lowercase()),
        }
    }

    fn parse(text: &str) -> Result<Self, Error> {
        let (category, name) = text
            .split_once(':')
            .ok_or_else(|| Error::Unsupported(text.into()))?;
        let mut chars = category.chars();
        let category = chars
            .next()
            .ok_or_else(|| Error::Unsupported(text.into()))?;
        if chars.next().is_some() {
            return Err(Error::Unsupported(text.into()));
        }
        let (name, unit) = name
            .split_once(',')
            .map_or((name, None), |(name, unit)| (name, Some(unit)));
        if name.trim().is_empty() {
            return Err(Error::Unsupported(text.into()));
        }
        Ok(Self::new(category, name, unit))
    }
}

pub trait Host {
    fn read(&mut self, reference: &Reference) -> Result<Value, Error>;
    fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error>;
}

#[derive(Debug)]
enum Instruction {
    Push(Value),
    Read(Reference),
    Write(Reference),
    Unary(fn(f64) -> f64),
    Binary(fn(f64, f64) -> f64),
    CompareString(bool),
    Duplicate,
    Pop,
    Swap,
    Clear,
    Store(usize, bool),
    Load(usize),
    Select,
    Case,
    JumpFalse(usize),
    Jump(usize),
}

#[derive(Debug)]
pub struct Program {
    instructions: Vec<Instruction>,
}

#[derive(Clone, Debug)]
enum Token<'a> {
    Word(&'a str),
    Variable(&'a str),
    Text(&'a str),
    Open,
    Close,
}

fn token<'a>(input: &mut &'a str) -> ModalResult<Token<'a>> {
    delimited(
        multispace0,
        alt((
            delimited('(', take_until(0.., ")"), ')').map(Token::Variable),
            delimited('\'', take_until(0.., "'"), '\'').map(Token::Text),
            '{'.value(Token::Open),
            '}'.value(Token::Close),
            take_while(1.., |c: char| !c.is_whitespace() && !"(){}'".contains(c)).map(Token::Word),
        )),
        multispace0,
    )
    .parse_next(input)
}

impl Program {
    pub fn compile(source: &str) -> Result<Self, Error> {
        let mut input = source.trim();
        let mut tokens = Vec::new();
        while !input.is_empty() {
            tokens.push(token(&mut input).map_err(|_| Error::Syntax(source.len() - input.len()))?);
        }
        let mut instructions = Vec::new();
        let mut blocks = Vec::new();
        let mut index = 0;
        while index < tokens.len() {
            match &tokens[index] {
                Token::Word("if") => {
                    if !matches!(tokens.get(index + 1), Some(Token::Open)) {
                        return Err(Error::Conditional);
                    }
                    blocks.push(instructions.len());
                    instructions.push(Instruction::JumpFalse(0));
                    index += 1;
                }
                Token::Close => {
                    let branch = blocks.pop().ok_or(Error::Conditional)?;
                    if matches!(tokens.get(index + 1), Some(Token::Word("els"))) {
                        if !matches!(tokens.get(index + 2), Some(Token::Open))
                            || !matches!(instructions[branch], Instruction::JumpFalse(_))
                        {
                            return Err(Error::Conditional);
                        }
                        let jump = instructions.len();
                        instructions.push(Instruction::Jump(0));
                        instructions[branch] = Instruction::JumpFalse(instructions.len());
                        blocks.push(jump);
                        index += 2;
                    } else {
                        match instructions[branch] {
                            Instruction::JumpFalse(_) => {
                                instructions[branch] = Instruction::JumpFalse(instructions.len())
                            }
                            Instruction::Jump(_) => {
                                instructions[branch] = Instruction::Jump(instructions.len())
                            }
                            _ => return Err(Error::Conditional),
                        }
                    }
                }
                Token::Open => return Err(Error::Conditional),
                Token::Text(text) => {
                    instructions.push(Instruction::Push(Value::Text((*text).into())))
                }
                Token::Variable(variable) => {
                    instructions.push(if let Some(variable) = variable.strip_prefix('>') {
                        Instruction::Write(Reference::parse(variable)?)
                    } else {
                        Instruction::Read(Reference::parse(variable)?)
                    });
                }
                Token::Word(word) => instructions.push(operation(word)?),
            }
            index += 1;
        }
        if !blocks.is_empty() {
            return Err(Error::Conditional);
        }
        Ok(Self { instructions })
    }

    pub fn evaluate(&self, host: &mut impl Host) -> Result<Option<Value>, Error> {
        let mut stack: Vec<Value> = Vec::new();
        let mut registers: [Value; 50] = std::array::from_fn(|_| Value::Number(0.0));
        let mut pc = 0;
        let pop = |stack: &mut Vec<Value>| stack.pop().ok_or(Error::Stack);
        while let Some(instruction) = self.instructions.get(pc) {
            match instruction {
                Instruction::Push(value) => stack.push(value.clone()),
                Instruction::Read(reference) => stack.push(host.read(reference)?),
                Instruction::Write(reference) => host.write(reference, pop(&mut stack)?)?,
                Instruction::Unary(op) => {
                    let a = pop(&mut stack)?.number()?;
                    stack.push(Value::Number(op(a)));
                }
                Instruction::Binary(op) => {
                    let b = pop(&mut stack)?.number()?;
                    let a = pop(&mut stack)?.number()?;
                    stack.push(Value::Number(op(a, b)));
                }
                Instruction::CompareString(insensitive) => {
                    let Value::Text(b) = pop(&mut stack)? else {
                        return Err(Error::Type);
                    };
                    let Value::Text(a) = pop(&mut stack)? else {
                        return Err(Error::Type);
                    };
                    let order = if *insensitive {
                        a.to_ascii_lowercase().cmp(&b.to_ascii_lowercase())
                    } else {
                        a.cmp(&b)
                    };
                    stack.push(Value::Number(match order {
                        std::cmp::Ordering::Less => -1.0,
                        std::cmp::Ordering::Equal => 0.0,
                        std::cmp::Ordering::Greater => 1.0,
                    }));
                }
                Instruction::Duplicate => stack.push(stack.last().ok_or(Error::Stack)?.clone()),
                Instruction::Pop => {
                    pop(&mut stack)?;
                }
                Instruction::Swap => {
                    let length = stack.len();
                    if length < 2 {
                        return Err(Error::Stack);
                    }
                    stack.swap(length - 1, length - 2);
                }
                Instruction::Clear => stack.clear(),
                Instruction::Store(index, remove) => {
                    registers[*index] = stack.last().ok_or(Error::Stack)?.clone();
                    if *remove {
                        pop(&mut stack)?;
                    }
                }
                Instruction::Load(index) => stack.push(registers[*index].clone()),
                Instruction::Select => {
                    let condition = pop(&mut stack)?.number()?;
                    let b = pop(&mut stack)?;
                    let a = pop(&mut stack)?;
                    stack.push(if condition != 0.0 { a } else { b });
                }
                Instruction::Case => {
                    let index = pop(&mut stack)?.number()?.floor();
                    let count = pop(&mut stack)?.number()?;
                    if count < 1.0
                        || count.fract() != 0.0
                        || count > stack.len() as f64
                        || index < 0.0
                        || index >= count
                    {
                        return Err(Error::Case);
                    }
                    let start = stack.len() - count as usize;
                    let value = stack[stack.len() - 1 - index as usize].clone();
                    stack.truncate(start);
                    stack.push(value);
                }
                Instruction::JumpFalse(target) => {
                    if pop(&mut stack)?.number()? == 0.0 {
                        pc = *target;
                        continue;
                    }
                }
                Instruction::Jump(target) => {
                    pc = *target;
                    continue;
                }
            }
            if let Some(Value::Number(value)) = stack.last()
                && !value.is_finite()
            {
                return Err(Error::Number);
            }
            pc += 1;
        }
        Ok(stack.pop())
    }
}

fn operation(word: &str) -> Result<Instruction, Error> {
    let lower = word.to_ascii_lowercase();
    let instruction = match lower.as_str() {
        "+" => Instruction::Binary(|a, b| a + b),
        "++" => Instruction::Unary(|a| a + 1.0),
        "--" => Instruction::Unary(|a| a - 1.0),
        "-" => Instruction::Binary(|a, b| a - b),
        "*" => Instruction::Binary(|a, b| a * b),
        "/" => Instruction::Binary(|a, b| a / b),
        "%" => Instruction::Binary(|a, b| a % b),
        "&" => Instruction::Binary(|a, b| bitwise(a, b, |a, b| a & b)),
        "|" => Instruction::Binary(|a, b| bitwise(a, b, |a, b| a | b)),
        "^" => Instruction::Binary(|a, b| bitwise(a, b, |a, b| a ^ b)),
        "min" => Instruction::Binary(f64::min),
        "max" => Instruction::Binary(f64::max),
        "pow" => Instruction::Binary(f64::powf),
        "log" => Instruction::Binary(f64::log),
        "div" => Instruction::Binary(|a, b| (a / b).trunc()),
        "==" => Instruction::Binary(|a, b| if a == b { 1.0 } else { 0.0 }),
        "!=" => Instruction::Binary(|a, b| if a != b { 1.0 } else { 0.0 }),
        "<" => Instruction::Binary(|a, b| if a < b { 1.0 } else { 0.0 }),
        ">" => Instruction::Binary(|a, b| if a > b { 1.0 } else { 0.0 }),
        "<=" => Instruction::Binary(|a, b| if a <= b { 1.0 } else { 0.0 }),
        ">=" => Instruction::Binary(|a, b| if a >= b { 1.0 } else { 0.0 }),
        "and" | "&&" => Instruction::Binary(|a, b| if a != 0.0 && b != 0.0 { 1.0 } else { 0.0 }),
        "or" | "||" => Instruction::Binary(|a, b| if a != 0.0 || b != 0.0 { 1.0 } else { 0.0 }),
        "!" | "not" => Instruction::Unary(|a| if a == 0.0 { 1.0 } else { 0.0 }),
        "/-/" | "neg" => Instruction::Unary(|a| -a),
        "abs" => Instruction::Unary(f64::abs),
        "sqrt" => Instruction::Unary(f64::sqrt),
        "sin" => Instruction::Unary(f64::sin),
        "cos" => Instruction::Unary(f64::cos),
        "tan" | "tg" => Instruction::Unary(f64::tan),
        "ctg" => Instruction::Unary(|a| 1.0 / a.tan()),
        "asin" => Instruction::Unary(f64::asin),
        "acos" => Instruction::Unary(f64::acos),
        "atan" | "atg" => Instruction::Unary(f64::atan),
        "atg2" => Instruction::Binary(f64::atan2),
        "dgrd" => Instruction::Unary(f64::to_radians),
        "rddg" => Instruction::Unary(f64::to_degrees),
        "flr" => Instruction::Unary(f64::floor),
        "int" => Instruction::Unary(f64::floor),
        "near" => Instruction::Unary(|a| (a + 0.5).floor()),
        "ceil" => Instruction::Unary(f64::ceil),
        "sqr" => Instruction::Unary(|a| a * a),
        "exp" => Instruction::Unary(f64::exp),
        "ln" => Instruction::Unary(f64::ln),
        "lg" => Instruction::Unary(f64::log10),
        "dnor" | "d360" | "rdeg" => Instruction::Unary(|a| a.rem_euclid(360.0)),
        "rnor" => Instruction::Unary(|a| a.rem_euclid(std::f64::consts::TAU)),
        "pi" => Instruction::Push(Value::Number(std::f64::consts::PI)),
        "d" => Instruction::Duplicate,
        "p" => Instruction::Pop,
        "r" => Instruction::Swap,
        "c" => Instruction::Clear,
        "scmp" => Instruction::CompareString(false),
        "scmi" => Instruction::CompareString(true),
        "?" => Instruction::Select,
        "case" => Instruction::Case,
        _ => {
            for (prefix, remove, load) in
                [("sp", true, false), ("s", false, false), ("l", false, true)]
            {
                if let Some(index) = word
                    .strip_prefix(prefix)
                    .and_then(|s| s.parse::<usize>().ok())
                    .filter(|&i| i < 50)
                {
                    return Ok(if load {
                        Instruction::Load(index)
                    } else {
                        Instruction::Store(index, remove)
                    });
                }
            }
            let number =
                if let Some(hex) = word.strip_prefix("0x").or_else(|| word.strip_prefix("0X")) {
                    u32::from_str_radix(hex, 16).map(f64::from).ok()
                } else if word.len() > 1
                    && word.starts_with('0')
                    && word.bytes().all(|b| b.is_ascii_digit())
                {
                    u32::from_str_radix(word, 8).map(f64::from).ok()
                } else {
                    word.parse::<f64>().ok()
                };
            let Some(value) = number else {
                return Err(Error::Unsupported(word.into()));
            };
            if !value.is_finite() {
                return Err(Error::Number);
            }
            Instruction::Push(Value::Number(value))
        }
    };
    Ok(instruction)
}

fn bitwise(a: f64, b: f64, operation: fn(i32, i32) -> i32) -> f64 {
    let integer = |value: f64| {
        (value >= f64::from(i32::MIN) && value <= f64::from(u32::MAX))
            .then_some(value.trunc() as i64 as i32)
    };
    match (integer(a), integer(b)) {
        (Some(a), Some(b)) => f64::from(operation(a, b)),
        _ => f64::NAN,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Default)]
    struct Variables(std::collections::HashMap<Reference, Value>);
    impl Host for Variables {
        fn read(&mut self, r: &Reference) -> Result<Value, Error> {
            self.0
                .get(r)
                .cloned()
                .ok_or_else(|| Error::Variable(r.name.clone()))
        }
        fn write(&mut self, r: &Reference, v: Value) -> Result<(), Error> {
            self.0.insert(r.clone(), v);
            Ok(())
        }
    }
    fn number(code: &str) -> f64 {
        Program::compile(code)
            .unwrap()
            .evaluate(&mut Variables::default())
            .unwrap()
            .unwrap()
            .number()
            .unwrap()
    }
    #[test]
    fn evaluates_nested_conditions_and_adjacent_braces() {
        assert_eq!(number("1 if{0 if{2}els{3}}els{4}"), 3.0);
        assert_eq!(number("0 if{1 0 /}els{7}"), 7.0);
        assert_eq!(number("0 if{ 99 } 3"), 3.0);
    }
    #[test]
    fn evaluates_sdk_stack_register_and_number_rules() {
        assert_eq!(number("022 0x10 + 5E-1 +"), 34.5);
        assert_eq!(number("1 2 3 s0 l0 + + +"), 9.0);
        assert_eq!(number("5 sp49 2 l49 *"), 10.0);
        assert_eq!(number("'left' 'Left' scmi 0 =="), 1.0);
        assert_eq!(number("10 20 1 ?"), 10.0);
        assert_eq!(number("-1.1 int"), -2.0);
        assert_eq!(number("-4.5 near"), -4.0);
        assert_eq!(number("50 40 30 20 10 5 2.5 case"), 30.0);
    }
    #[test]
    fn reads_and_writes_variables_without_silent_defaults() {
        let program = Program::compile("42 (>L:VALUE,number) (L:value, number) 2 /").unwrap();
        assert_eq!(
            program.evaluate(&mut Variables::default()).unwrap(),
            Some(Value::Number(21.0))
        );
        assert!(
            Program::compile("(A:missing,bool)")
                .unwrap()
                .evaluate(&mut Variables::default())
                .is_err()
        );
    }
    #[test]
    fn rejects_malformed_unsupported_and_invalid_execution() {
        for code in ["1 if{", "}", "els{1}", "(A:x", "'x", "s50", "NaN", "while"] {
            assert!(Program::compile(code).is_err(), "{code}");
        }
        for code in ["+", "1 0 /", "'x' 1 +", "-1 sqrt", "1 if{p}"] {
            assert!(
                Program::compile(code)
                    .unwrap()
                    .evaluate(&mut Variables::default())
                    .is_err(),
                "{code}"
            );
        }
    }
}
