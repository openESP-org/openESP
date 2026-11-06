use crate::interaction::*;
use crate::variables::Variables;
use esp_assets::ModelMouseRectangle;
use esp_assets::{Model, gauge::MouseEvent};
use esp_script::Value;
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
