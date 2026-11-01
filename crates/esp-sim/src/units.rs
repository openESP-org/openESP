use esp_script::Value;

pub fn convert(value: &Value, from: Option<&str>, to: Option<&str>) -> Option<Value> {
    let (from, to) = (from?, to?);
    let unit = |name| match name {
        "bool" | "boolean" | "number" | "percent over 100" => Some((0, 1.0)),
        "percent" => Some((0, 0.01)),
        "degrees" | "degree" => Some((1, std::f64::consts::PI / 180.0)),
        "radians" | "radian" => Some((1, 1.0)),
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
