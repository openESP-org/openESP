#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("model parameter is not UTF-8 text")]
    Encoding(#[from] std::str::Utf8Error),
    #[error("invalid model parameter lag")]
    Lag,
    #[error("model parameter contains an embedded null")]
    Null,
}

#[derive(Debug, PartialEq)]
pub struct Code<'a> {
    pub expression: &'a str,
    pub lag: Option<f64>,
}

impl<'a> Code<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        let text = std::str::from_utf8(bytes.strip_suffix(&[0]).unwrap_or(bytes))?;
        if text.contains('\0') {
            return Err(Error::Null);
        }
        if let Some(header) = text.strip_prefix('#') {
            let (lag, expression) = header.split_once('#').ok_or(Error::Lag)?;
            let lag = lag.parse::<f64>().map_err(|_| Error::Lag)?;
            if !lag.is_finite() || lag < 0.0 {
                return Err(Error::Lag);
            }
            Ok(Self {
                expression,
                lag: Some(lag),
            })
        } else {
            Ok(Self {
                expression: text,
                lag: None,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn separates_optional_lag_without_changing_expression() {
        assert_eq!(
            Code::parse(b"#50#\r\n 1 \0").unwrap(),
            Code {
                lag: Some(50.0),
                expression: "\r\n 1 "
            }
        );
        assert_eq!(Code::parse(b"1").unwrap().lag, None);
        for bytes in [b"#-1#x".as_slice(), b"#NaN#x", b"#50", b"x\0y", &[255]] {
            assert!(Code::parse(bytes).is_err());
        }
    }
}
