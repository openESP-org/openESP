use esp_script::Value;

pub fn frequency_from_bcd16(value: f64) -> Option<f64> {
    if !value.is_finite() || value.fract() != 0.0 || !(0.0..=65535.0).contains(&value) {
        return None;
    }
    let encoded = value as u16;
    let digits = [
        (encoded >> 12) & 15,
        (encoded >> 8) & 15,
        (encoded >> 4) & 15,
        encoded & 15,
    ];
    if digits.iter().any(|digit| *digit > 9) {
        return None;
    }
    let decimal =
        digits[0] as u32 * 1000 + digits[1] as u32 * 100 + digits[2] as u32 * 10 + digits[3] as u32;
    let extra = if matches!(digits[3], 2 | 7) { 5_000 } else { 0 };
    Some((100_000_000 + decimal * 10_000 + extra) as f64)
}

fn frequency_to_bcd16(hz: f64) -> Option<f64> {
    if !hz.is_finite() || !(100_000_000.0..200_000_000.0).contains(&hz) {
        return None;
    }
    let decimal = ((hz - 100_000_000.0 + 0.01) / 10_000.0).floor() as u32;
    let encoded = ((decimal / 1000) << 12)
        | (((decimal / 100) % 10) << 8)
        | (((decimal / 10) % 10) << 4)
        | (decimal % 10);
    Some(encoded as f64)
}

pub fn convert(value: &Value, from: Option<&str>, to: Option<&str>) -> Option<Value> {
    let (from, to) = (from?, to?);
    let unit = |name| match name {
        "bool" | "boolean" | "number" | "enum" | "part" | "position" | "percent over 100"
        | "mach" => Some((0, 1.0)),
        "percent" | "keyframe" => Some((0, 0.01)),
        "degrees" | "degree" => Some((1, std::f64::consts::PI / 180.0)),
        "grad" | "grads" => Some((1, std::f64::consts::PI / 200.0)),
        "radians" | "radian" => Some((1, 1.0)),
        "knots" | "knot" => Some((2, 1852.0 / 3600.0)),
        "meters per second" => Some((2, 1.0)),
        "feet per second" => Some((2, 0.3048)),
        "feet per minute" => Some((2, 0.3048 / 60.0)),
        "meters" | "meter" => Some((3, 1.0)),
        "feet" | "foot" => Some((3, 0.3048)),
        "nautical miles" | "nautical mile" | "nautical miles long" => Some((3, 1852.0)),
        "pascals" | "pascal" => Some((4, 1.0)),
        "millibars" | "millibar" | "hectopascals" | "hectopascal" | "hpa" => Some((4, 100.0)),
        "inhg" => Some((4, 3386.389)),
        "psi" | "pounds per square inch" => Some((4, 6894.757293168)),
        "psf" | "pounds per square foot" => Some((4, 47.88025898033584)),
        "kilograms" | "kilogram" => Some((5, 1.0)),
        "pounds" | "pound" => Some((5, 0.45359237)),
        "gallons" | "gallon" => Some((6, 0.003785411784)),
        "liters" | "liter" => Some((6, 0.001)),
        "newtons" => Some((7, 1.0)),
        "pounds force" => Some((7, 4.4482216152605)),
        "kilograms per second" => Some((8, 1.0)),
        "pounds per hour" => Some((8, 0.45359237 / 3600.0)),
        "rpm" | "revolutions per minute" => Some((9, 1.0)),
        "watts" => Some((10, 1.0)),
        "horsepower" => Some((10, 745.69987158227)),
        "hertz" | "hz" => Some((11, 1.0)),
        "kilohertz" | "khz" => Some((11, 1000.0)),
        "megahertz" | "mhz" => Some((11, 1_000_000.0)),
        "radians per second" | "radian per second" => Some((12, 1.0)),
        "degrees per second" | "degree per second" => Some((12, std::f64::consts::PI / 180.0)),
        _ => None,
    };
    if from == "frequency bcd16" {
        let hz = frequency_from_bcd16(value.number().ok()?)?;
        let (dimension, scale) = unit(to)?;
        return (dimension == 11).then_some(Value::Number(hz / scale));
    }
    if to == "frequency bcd16" {
        let (dimension, scale) = unit(from)?;
        return (dimension == 11)
            .then(|| frequency_to_bcd16(value.number().ok()? * scale).map(Value::Number))
            .flatten();
    }
    let (from_dimension, from_scale) = unit(from)?;
    let (to_dimension, to_scale) =
        unit(if from_dimension == 7 && matches!(to, "pounds" | "pound") {
            "pounds force"
        } else {
            to
        })?;
    if from_dimension != to_dimension {
        return None;
    }
    let value = value.number().ok()? * from_scale / to_scale;
    let value = if matches!(to, "bool" | "boolean") {
        f64::from(value != 0.0)
    } else {
        value
    };
    value.is_finite().then_some(Value::Number(value))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bcd_frequency_converts_with_twenty_five_kilohertz_precision() {
        for (hz, encoded) in [
            (108_000_000.0, 0x0800),
            (113_025_000.0, 0x1302),
            (113_075_000.0, 0x1307),
            (117_950_000.0, 0x1795),
        ] {
            assert_eq!(
                convert(&Value::Number(hz), Some("hertz"), Some("frequency bcd16")),
                Some(Value::Number(encoded as f64))
            );
            assert_eq!(
                convert(
                    &Value::Number(encoded as f64),
                    Some("frequency bcd16"),
                    Some("hertz")
                ),
                Some(Value::Number(hz))
            );
        }
        assert!(frequency_from_bcd16(0x1a00 as f64).is_none());
        assert!(
            convert(
                &Value::Number(10.0),
                Some("meters"),
                Some("frequency bcd16")
            )
            .is_none()
        );
    }
}
