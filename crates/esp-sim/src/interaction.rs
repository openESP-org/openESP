use anyhow::{Context, Result};
use esp_assets::{Model, gauge::MouseEvent};
use esp_script::Value;

use crate::variables::Variables;

pub fn model_mouse(
    model: &Model,
    rectangle: usize,
    event: MouseEvent,
    variables: &mut Variables,
) -> Result<bool> {
    let rectangle = model
        .mouse_rectangles
        .get(rectangle)
        .context("invalid model mouse rectangle")?;
    let flag = match event {
        MouseEvent::LeftSingle => 1 << 29,
        MouseEvent::LeftRelease => 1 << 17,
        MouseEvent::LeftDrag => 1 << 23,
        MouseEvent::DownRepeat => {
            if rectangle.flags & (1 << 29) == 0 {
                return Ok(false);
            }
            1 << 21
        }
        MouseEvent::WheelUp => 1 << 14,
        MouseEvent::WheelDown => 1 << 13,
    };
    if rectangle.flags & flag == 0 {
        return Ok(false);
    }
    let id = rectangle
        .key_event
        .context("unsupported model mouse callback")?;
    variables.dispatch_key_event(id, Value::Number(0.0))?;
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_assets::ModelMouseRectangle;
    use esp_script::{Host, Reference};

    #[test]
    fn filters_mouse_flags_and_rejects_unsupported_callbacks() {
        let mut model = Model {
            nodes: vec![],
            materials: vec![],
            lods: vec![],
            animations: vec![],
            visibility: vec![],
            mouse_rectangles: vec![ModelMouseRectangle {
                flags: (1 << 29) | (1 << 21),
                key_event: Some(0x102f0),
            }],
        };
        let mut variables = Variables::default();
        let reference = Reference::new('A', "AUTOPILOT FLIGHT DIRECTOR ACTIVE", Some("bool"));
        assert!(!model_mouse(&model, 0, MouseEvent::WheelUp, &mut variables).unwrap());
        assert!(variables.read(&reference).is_err());
        assert!(model_mouse(&model, 0, MouseEvent::LeftSingle, &mut variables).unwrap());
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(1.0));
        assert!(model_mouse(&model, 0, MouseEvent::DownRepeat, &mut variables).unwrap());
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
        model.mouse_rectangles[0].flags &= !(1 << 21);
        assert!(!model_mouse(&model, 0, MouseEvent::DownRepeat, &mut variables).unwrap());
        model.mouse_rectangles[0].key_event = None;
        assert!(model_mouse(&model, 0, MouseEvent::LeftSingle, &mut variables).is_err());
        model.mouse_rectangles[0].key_event = Some(u32::MAX);
        assert!(model_mouse(&model, 0, MouseEvent::LeftSingle, &mut variables).is_err());
        assert!(model_mouse(&model, 1, MouseEvent::LeftSingle, &mut variables).is_err());
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
        assert_eq!(variables.drain_events().count(), 0);
    }
}
