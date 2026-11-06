use anyhow::{Context, Result};
use esp_assets::{Model, gauge::MouseEvent};
use esp_script::Value;

use crate::variables::VariableStore;

pub fn model_mouse(
    model: &Model,
    rectangle: usize,
    event: MouseEvent,
    variables: &mut dyn VariableStore,
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
