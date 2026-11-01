use esp_script::Value;

pub fn convert(value: &Value, from: Option<&str>, to: Option<&str>) -> Option<Value> {
    let (from, to) = (from?, to?);
    let unit = |name| match name {
        "bool" | "boolean" | "number" | "percent over 100" | "mach" => Some((0, 1.0)),
        "percent" => Some((0, 0.01)),
        "degrees" | "degree" => Some((1, std::f64::consts::PI / 180.0)),
        "radians" | "radian" => Some((1, 1.0)),
        "knots" | "knot" => Some((2, 1852.0 / 3600.0)),
        "meters per second" => Some((2, 1.0)),
        "feet per second" => Some((2, 0.3048)),
        "feet per minute" => Some((2, 0.3048 / 60.0)),
        "meters" | "meter" => Some((3, 1.0)),
        "feet" | "foot" => Some((3, 0.3048)),
        "pascals" | "pascal" => Some((4, 1.0)),
        "millibars" => Some((4, 100.0)),
        "inhg" => Some((4, 3386.389)),
        _ => None,
    };
    let (from_dimension, from_scale) = unit(from)?;
    let (to_dimension, to_scale) = unit(to)?;
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
