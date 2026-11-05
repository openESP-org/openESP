use crate::{Error, Host, Program, Value};

#[derive(Debug)]
pub struct FormattedString(Vec<Part>);

#[derive(Debug)]
enum Part {
    Literal(String),
    Value(Program, Option<Format>),
    If(Program, Vec<Part>, Vec<Part>),
    Case(Program, Vec<(i32, Vec<Part>)>),
}

#[derive(Debug)]
struct Format {
    kind: char,
    width: usize,
    precision: Option<usize>,
    zero: bool,
    left: bool,
    plus: bool,
}

struct Parser<'a> {
    text: &'a str,
    position: usize,
}

#[derive(PartialEq)]
enum Stop {
    End,
    Else,
    Case(i32),
    Eof,
}

impl FormattedString {
    pub fn compile(text: &str) -> Result<Self, Error> {
        if text.len() > 65536 {
            return Err(Error::Unsupported("gauge string exceeds size limit".into()));
        }
        let (parts, stop) = Parser { text, position: 0 }.block(0)?;
        if stop != Stop::Eof {
            return Err(Error::Conditional);
        }
        Ok(Self(parts))
    }

    pub fn evaluate(&self, host: &mut impl Host) -> Result<String, Error> {
        let mut result = String::new();
        evaluate(&self.0, host, &mut result)?;
        Ok(result)
    }
}

fn evaluate(parts: &[Part], host: &mut impl Host, result: &mut String) -> Result<(), Error> {
    let value = |program: &Program, host: &mut _| program.evaluate(host)?.ok_or(Error::Stack);
    for part in parts {
        match part {
            Part::Literal(text) => result.push_str(text),
            Part::Value(program, format) => {
                let value = value(program, host)?;
                result.push_str(&if let Some(format) = format {
                    format.apply(value)?
                } else {
                    display(value)?
                });
            }
            Part::If(program, yes, no) => evaluate(
                if value(program, host)?.number()? != 0.0 {
                    yes
                } else {
                    no
                },
                host,
                result,
            )?,
            Part::Case(program, cases) => {
                let value = value(program, host)?.number()?;
                if let Some((_, parts)) = cases.iter().find(|(index, _)| f64::from(*index) == value)
                {
                    evaluate(parts, host, result)?;
                }
            }
        }
        if result.len() > 16384 {
            return Err(Error::Unsupported(
                "formatted gauge string exceeds size limit".into(),
            ));
        }
    }
    Ok(())
}

fn display(value: Value) -> Result<String, Error> {
    match value {
        Value::Text(text) => Ok(text),
        Value::Number(value) if value.is_finite() => Ok(value.to_string()),
        _ => Err(Error::Number),
    }
}

impl Format {
    fn parse(text: &str) -> Result<Self, Error> {
        let fail = || Error::Unsupported(format!("gauge number format {text}"));
        let kind = text.chars().last().ok_or_else(fail)?;
        if !matches!(kind, 's' | 'd' | 'f') {
            return Err(fail());
        }
        let mut prefix = &text[..text.len() - 1];
        let left = prefix.starts_with('-');
        let plus = prefix.starts_with('+');
        if left || plus || prefix.starts_with(' ') {
            prefix = &prefix[1..];
        }
        let zero = prefix.starts_with('0');
        let (width, precision) = prefix
            .split_once('.')
            .map_or((prefix, None), |(w, p)| (w, Some(p)));
        if !width.bytes().all(|b| b.is_ascii_digit())
            || precision.is_some_and(|p| p.is_empty() || !p.bytes().all(|b| b.is_ascii_digit()))
        {
            return Err(fail());
        }
        let width = if width.is_empty() {
            0
        } else {
            width.parse::<usize>().map_err(|_| fail())?
        };
        let precision = precision
            .map(|p| p.parse::<usize>().map_err(|_| fail()))
            .transpose()?;
        if width > 512 || precision.is_some_and(|p| p > 32) {
            return Err(fail());
        }
        Ok(Self {
            kind,
            width,
            precision,
            zero,
            left,
            plus,
        })
    }

    fn apply(&self, value: Value) -> Result<String, Error> {
        let text = match self.kind {
            's' => display(value)?,
            'd' => {
                let number = value.number()?.round();
                let number = if number == 0.0 { 0.0 } else { number };
                if self.plus {
                    format!("{number:+.0}")
                } else {
                    format!("{number:.0}")
                }
            }
            'f' => {
                let number = value.number()?;
                let precision = self.precision.unwrap_or(6);
                return Ok(if self.plus {
                    format!("{number:+.precision$}")
                } else {
                    format!("{number:.precision$}")
                });
            }
            _ => return Err(Error::Type),
        };
        let padding = self.width.saturating_sub(text.chars().count());
        if self.left {
            return Ok(format!("{text}{}", " ".repeat(padding)));
        }
        if self.zero && self.kind == 'd' {
            if text.starts_with(['-', '+']) {
                return Ok(format!(
                    "{}{}{}",
                    &text[..1],
                    "0".repeat(padding),
                    &text[1..]
                ));
            }
            return Ok(format!("{}{text}", "0".repeat(padding)));
        }
        Ok(format!("{}{text}", " ".repeat(padding)))
    }
}

impl Parser<'_> {
    fn rest(&self) -> &str {
        &self.text[self.position..]
    }

    fn block(&mut self, depth: usize) -> Result<(Vec<Part>, Stop), Error> {
        if depth > 32 {
            return Err(Error::Conditional);
        }
        let mut parts = Vec::new();
        while self.position < self.text.len() {
            let rest = self.rest();
            if rest.starts_with("%{") {
                let end = rest.find('}').ok_or(Error::Conditional)?;
                let command = rest[2..end].trim();
                let stop = match command {
                    "else" => Stop::Else,
                    "end" => Stop::End,
                    command if command.starts_with(':') => Stop::Case(
                        command[1..]
                            .trim()
                            .parse()
                            .map_err(|_| Error::Conditional)?,
                    ),
                    _ => {
                        return Err(Error::Unsupported(format!(
                            "gauge string command {command}"
                        )));
                    }
                };
                self.position += end + 1;
                return Ok((parts, stop));
            }
            if rest.starts_with("%%") {
                parts.push(Part::Literal("%".into()));
                self.position += 2;
                continue;
            }
            if rest.starts_with("%(") {
                let program = self.expression()?;
                // Gauge authors often put control directives on the next line.
                // Only consume the separator when a directive actually follows.
                let rest = self.rest();
                let trimmed = rest.trim_start_matches(char::is_whitespace);
                if trimmed.starts_with("%{if}") || trimmed.starts_with("%{case}") {
                    self.position += rest.len() - trimmed.len();
                }
                if self.rest().starts_with("%{if}") {
                    self.position += 5;
                    let (yes, stop) = self.block(depth + 1)?;
                    let no = if stop == Stop::Else {
                        let (no, stop) = self.block(depth + 1)?;
                        if stop != Stop::End {
                            return Err(Error::Conditional);
                        }
                        no
                    } else {
                        if stop != Stop::End {
                            return Err(Error::Conditional);
                        }
                        Vec::new()
                    };
                    parts.push(Part::If(program, yes, no));
                } else if self.rest().starts_with("%{case}") {
                    self.position += 7;
                    let (before, mut stop) = self.block(depth + 1)?;
                    if before
                        .iter()
                        .any(|p| !matches!(p,Part::Literal(t) if t.trim().is_empty()))
                    {
                        return Err(Error::Conditional);
                    }
                    let mut cases = Vec::new();
                    while let Stop::Case(index) = stop {
                        if cases.iter().any(|(i, _)| *i == index) {
                            return Err(Error::Conditional);
                        }
                        let (body, next) = self.block(depth + 1)?;
                        cases.push((index, body));
                        stop = next;
                    }
                    if stop != Stop::End {
                        return Err(Error::Conditional);
                    }
                    parts.push(Part::Case(program, cases));
                } else {
                    let format = if self.rest().starts_with("%!") {
                        self.position += 2;
                        let end = self.rest().find('!').ok_or(Error::Syntax(self.position))?;
                        let format = Format::parse(&self.rest()[..end])?;
                        self.position += end + 1;
                        Some(format)
                    } else {
                        None
                    };
                    parts.push(Part::Value(program, format));
                }
                continue;
            }
            let end = rest.find('%').unwrap_or(rest.len());
            if end == 0 {
                return Err(Error::Syntax(self.position));
            }
            let literal = &rest[..end];
            let directive = &rest[end..];
            // Indentation between a case body and the next directive is XML
            // formatting, not another line in the instrument display.
            let indentation = literal.trim().is_empty()
                && literal.contains(['\r', '\n'])
                && (directive.starts_with("%{:")
                    || directive.starts_with("%{end}")
                    || directive.starts_with("%{else}"));
            if !indentation {
                parts.push(Part::Literal(literal.into()));
            }
            self.position += end;
        }
        Ok((parts, Stop::Eof))
    }

    fn expression(&mut self) -> Result<Program, Error> {
        self.position += 2;
        let start = self.position;
        let mut depth = 1;
        let mut quoted = false;
        while self.position < self.text.len() {
            if !quoted && self.rest().starts_with("/*") {
                let end = self.rest().find("*/").ok_or(Error::Syntax(self.position))?;
                self.position += end + 2;
                continue;
            }
            let character = self.rest().chars().next().unwrap();
            if character == '\'' {
                quoted = !quoted;
            }
            if !quoted {
                match character {
                    '(' => depth += 1,
                    ')' => depth -= 1,
                    _ => {}
                }
            }
            if depth == 0 {
                let program = Program::compile(&self.text[start..self.position])?;
                self.position += 1;
                return Ok(program);
            }
            self.position += character.len_utf8();
        }
        Err(Error::Syntax(start))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Reference;

    struct Empty;
    impl Host for Empty {
        fn read(&mut self, _: &Reference) -> Result<Value, Error> {
            Err(Error::Variable("missing".into()))
        }
        fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
            Err(Error::Type)
        }
    }

    #[test]
    fn follows_sdk_decimal_float_and_string_formats() {
        for (source, expected) in [
            ("%(12.34)%!4.3f!", "12.340"),
            ("%(12.34)%!04.3f!", "12.340"),
            ("%(34.56)%!+d!", "+35"),
            ("%(234)%!5d!", "  234"),
            ("%('foo')%!5s!", "  foo"),
            ("%(234)%!3s!", "234"),
            ("%(-5)%!04d!", "-005"),
            ("%(2)%!-4d!", "2   "),
            ("%(9)%!03d!", "009"),
            ("A=%%; %('literal (text)')%!s!", "A=%; literal (text)"),
        ] {
            assert_eq!(
                FormattedString::compile(source)
                    .unwrap()
                    .evaluate(&mut Empty)
                    .unwrap(),
                expected,
                "{source}"
            );
        }
    }

    #[test]
    fn evaluates_only_selected_branches_and_nested_conditionals() {
        let source =
            "%(1)%{if}ON %(0)%{if}bad%{else}OK%{end}%{else}%((A:MISSING,number))%!d!%{end}";
        assert_eq!(
            FormattedString::compile(source)
                .unwrap()
                .evaluate(&mut Empty)
                .unwrap(),
            "ON OK"
        );
        assert_eq!(
            FormattedString::compile("%(0)%{if}hidden%{end}shown")
                .unwrap()
                .evaluate(&mut Empty)
                .unwrap(),
            "shown"
        );
        assert_eq!(
            FormattedString::compile("%(3)%{case}%{ :0 }AIRPORT%{ :3 }VOR%{end}")
                .unwrap()
                .evaluate(&mut Empty)
                .unwrap(),
            "VOR"
        );
        assert_eq!(
            FormattedString::compile("%(1)\n  %{case}\n%{:0}OFF%{:1}ON%{end}")
                .unwrap()
                .evaluate(&mut Empty)
                .unwrap(),
            "ON"
        );
        assert_eq!(
            FormattedString::compile(
                "%(1)\n%{case}\n%{:0}0.0\n%{:1} %(4.2)%!4.1f!\n\t%{:2}0.0\n%{end}"
            )
            .unwrap()
            .evaluate(&mut Empty)
            .unwrap(),
            " 4.2"
        );
        assert_eq!(
            FormattedString::compile("%(1)  %{if}ON%{else}OFF%{end}")
                .unwrap()
                .evaluate(&mut Empty)
                .unwrap(),
            "ON"
        );
        assert_eq!(
            FormattedString::compile("%(1)  units")
                .unwrap()
                .evaluate(&mut Empty)
                .unwrap(),
            "1  units"
        );
    }

    #[test]
    fn rejects_malformed_or_excessive_formatting() {
        for source in [
            "%((1)",
            "%{else}",
            "%(1)%{if}missing end",
            "%(1)%!999999d!",
            "%(1)%!3.1000f!",
            "%(1)%!x!",
            "%(1)%{case}%{:0}a%{:0}b%{end}",
        ] {
            assert!(FormattedString::compile(source).is_err(), "{source}");
        }
        assert!(
            FormattedString::compile("%((A:MISSING,number))%!d!")
                .unwrap()
                .evaluate(&mut Empty)
                .is_err()
        );
    }
}
