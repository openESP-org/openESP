use esp_script::{Error, Host, Reference, Value};
use std::collections::HashMap;

#[derive(Default)]
pub struct Variables {
    values: HashMap<(char, String), Variable>,
    events: Vec<(String, Value)>,
    flaps: Option<crate::flaps::Flaps>,
    engines: Vec<u32>,
    electrical: Option<crate::electrical::Electrical>,
}

struct Variable {
    unit: Option<String>,
    value: Value,
}

fn variable_name(category: char, name: &str) -> String {
    if category == 'A' && !name.contains(':') {
        for prefix in ["TURB ENG", "RECIP ENG", "GENERAL ENG", "ENG", "PROP"] {
            if let Some(rest) = name.strip_prefix(prefix) {
                let count = rest.bytes().take_while(|b| b.is_ascii_digit()).count();
                if count > 0
                    && let Some(suffix) = rest[count..].strip_prefix(' ')
                    && let Ok(index) = rest[..count].parse::<u32>()
                    && index > 0
                {
                    return format!("{prefix} {suffix}:{index}");
                }
            }
        }
    }
    name.into()
}

impl Variables {
    pub fn configure_electrical(
        &mut self,
        definition: &esp_formats::cfg::electrical::Electrical,
        running: bool,
    ) -> anyhow::Result<()> {
        let electrical = crate::electrical::Electrical::new(definition.clone());
        electrical.initialize(self, running);
        self.electrical = Some(electrical);
        self.update_electrical()
    }

    pub fn update_electrical(&mut self) -> anyhow::Result<()> {
        if let Some(electrical) = self.electrical.take() {
            let result = electrical.update(self);
            self.electrical = Some(electrical);
            result?;
        }
        Ok(())
    }

    pub fn configure_engines(&mut self, indices: impl Iterator<Item = u32>) -> anyhow::Result<()> {
        let mut engines = indices.collect::<Vec<_>>();
        engines.sort_unstable();
        anyhow::ensure!(
            engines.iter().all(|i| *i > 0) && engines.windows(2).all(|p| p[0] != p[1]),
            "invalid engine indices"
        );
        for index in &engines {
            self.set_number(
                &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
                "percent",
                0.0,
            );
            self.set_number(&format!("GENERAL ENG FUEL VALVE:{index}"), "bool", 1.0);
        }
        self.engines = engines;
        Ok(())
    }

    pub fn flap_angles(&self) -> impl Iterator<Item = f64> + '_ {
        self.flaps.iter().flat_map(|flaps| flaps.angles())
    }

    pub fn configure_flaps(
        &mut self,
        definitions: &[esp_formats::cfg::aircraft::FlapSet],
    ) -> anyhow::Result<()> {
        self.flaps = Some(crate::flaps::Flaps::new(definitions)?);
        Ok(())
    }

    pub fn advance(&mut self, dt: f64) -> anyhow::Result<()> {
        anyhow::ensure!(dt.is_finite() && dt >= 0.0, "invalid simulation timestep");
        self.update_electrical()?;
        let reference = Reference::new('A', "AIRSPEED INDICATED", Some("knots"));
        let airspeed = if self.values.contains_key(&('A', reference.name.clone())) {
            self.read(&reference)?.number()?
        } else {
            0.0
        };
        if let Some(flaps) = &mut self.flaps {
            flaps.advance(dt, airspeed)?;
        }
        Ok(())
    }

    pub fn interpolate(&mut self, alpha: f64) -> anyhow::Result<()> {
        anyhow::ensure!(
            alpha.is_finite() && (0.0..=1.0).contains(&alpha),
            "invalid simulation interpolation"
        );
        if let Some(flaps) = &mut self.flaps {
            flaps.interpolate(alpha)?;
        }
        Ok(())
    }

    pub fn set(&mut self, category: char, name: &str, unit: Option<&str>, value: Value) {
        let reference = Reference::new(category, name, unit);
        self.values.insert(
            (
                reference.category,
                variable_name(reference.category, &reference.name),
            ),
            Variable {
                unit: reference.unit,
                value,
            },
        );
    }

    pub fn set_number(&mut self, name: &str, unit: &str, value: f64) {
        self.set('A', name, Some(unit), Value::Number(value));
    }

    pub fn drain_events(&mut self) -> impl Iterator<Item = (String, Value)> + '_ {
        self.events.drain(..)
    }

    pub fn dispatch_events(&mut self) -> Vec<Error> {
        std::mem::take(&mut self.events)
            .into_iter()
            .filter_map(|(name, value)| self.dispatch_event(&name, value).err())
            .collect()
    }

    pub fn dispatch_key_event(&mut self, id: u32, value: Value) -> Result<(), Error> {
        if (0x10187..=0x1019e).contains(&id) {
            let offset = id - 0x10187;
            let action = match offset % 6 {
                0 => "OFF",
                1 => "RIGHT",
                2 => "LEFT",
                3 => "BOTH",
                4 => "START",
                5 => return self.dispatch_event(&format!("STARTER{}_SET", offset / 6 + 1), value),
                _ => return Err(Error::Unsupported(format!("key event {id:#x}"))),
            };
            return self.dispatch_event(&format!("MAGNETO{}_{action}", offset / 6 + 1), value);
        }
        let name = match id {
            0x10018 => "STROBES_TOGGLE",
            0x10022 => "GEAR_TOGGLE",
            0x10035 => "SPOILERS_TOGGLE",
            0x1003b => "FLAPS_UP",
            0x10043 => "FLAPS_DOWN",
            0x100a4 => "GEAR_SET",
            0x1021f => "GEAR_UP",
            0x10220 => "GEAR_DOWN",
            0x10047 => "ELEV_TRIM_DN",
            0x1004f => "ELEV_TRIM_UP",
            0x100aa => "ELEVATOR_TRIM_SET",
            0x100e6 => "AXIS_ELEV_TRIM_SET",
            0x100a2 => "FLAPS_SET",
            0x100de => "FLAPS_INCR",
            0x100df => "FLAPS_DECR",
            0x100d7 => "LANDING_LIGHTS_TOGGLE",
            0x100d8 => "PARKING_BRAKES",
            0x100ed => "MIXTURE_SET",
            0x100ee => "MIXTURE_RICH",
            0x100ef => "MIXTURE_INCR",
            0x100f0 => "MIXTURE_INCR_SMALL",
            0x100f1 => "MIXTURE_DECR",
            0x100f2 => "MIXTURE_LEAN",
            0x100fa => "SPOILERS_SET",
            0x10142 => "PITOT_HEAT_TOGGLE",
            0x10157 => "HEADING_BUG_INC",
            0x10158 => "HEADING_BUG_DEC",
            0x1017f => "MIXTURE1_SET",
            0x10180 => "MIXTURE2_SET",
            0x10181 => "MIXTURE3_SET",
            0x10182 => "MIXTURE4_SET",
            0x101e7 => "MAGNETO_OFF",
            0x101e8 => "MAGNETO_RIGHT",
            0x101e9 => "MAGNETO_LEFT",
            0x101ea => "MAGNETO_BOTH",
            0x101eb => "MAGNETO_START",
            0x102bf => "TOGGLE_BEACON_LIGHTS",
            0x10210 => "SPOILERS_ON",
            0x10211 => "SPOILERS_OFF",
            0x102c0 => "TOGGLE_TAXI_LIGHTS",
            0x102c1 => "TOGGLE_MASTER_BATTERY",
            0x102c2 => "TOGGLE_MASTER_ALTERNATOR",
            0x102f0 => "TOGGLE_FLIGHT_DIRECTOR",
            0x102f4 => "AXIS_MIXTURE_SET",
            0x102f5 => "TOGGLE_AVIONICS_MASTER",
            0x102fc => "TOGGLE_STARTER1",
            0x102fd => "TOGGLE_STARTER2",
            0x102fe => "TOGGLE_STARTER3",
            0x102ff => "TOGGLE_STARTER4",
            0x1034b => "TOGGLE_NAV_LIGHTS",
            0x1034e => "AXIS_SPOILER_SET",
            0x1033b => "TOGGLE_ALTERNATOR1",
            0x1033c => "TOGGLE_ALTERNATOR2",
            0x1033d => "TOGGLE_ALTERNATOR3",
            0x1033e => "TOGGLE_ALTERNATOR4",
            0x10346 => "TOGGLE_MASTER_BATTERY_ALTERNATOR",
            0x1035f => "MAGNETO_SET_ACTUAL",
            0x10360 => "MAGNETO1_SET",
            0x10361 => "MAGNETO2_SET",
            0x10362 => "MAGNETO3_SET",
            0x10363 => "MAGNETO4_SET",
            0x10712 => "GENALT_BUS1_CONTACT_SET",
            0x10713 => "GENALT_BUS2_CONTACT_SET",
            0x10714 => "GENALT_BUS3_CONTACT_SET",
            0x10715 => "GENALT_BUS4_CONTACT_SET",
            0x10716 => "TOGGLE_GENALT_BUS1_CONTACT",
            0x10717 => "TOGGLE_GENALT_BUS2_CONTACT",
            0x10718 => "TOGGLE_GENALT_BUS3_CONTACT",
            0x10719 => "TOGGLE_GENALT_BUS4_CONTACT",
            0x1071a => "BATTERY_BUS_CONTACT_SET",
            0x1071b => "TOGGLE_BATTERY_BUS_CONTACT",
            _ => return Err(Error::Unsupported(format!("key event {id:#x}"))),
        };
        self.dispatch_event(name, value)
    }

    fn toggle(&mut self, name: &str) -> Result<(), Error> {
        let reference = Reference::new('A', name, Some("bool"));
        let active = if self.values.contains_key(&('A', reference.name.clone())) {
            self.read(&reference)?.number()?
        } else {
            0.0
        };
        self.set_number(name, "bool", f64::from(active == 0.0));
        Ok(())
    }

    pub fn dispatch_event(&mut self, name: &str, value: Value) -> Result<(), Error> {
        let value = value.number()?;
        if let Some(index) = name
            .strip_prefix("STARTER")
            .and_then(|s| s.strip_suffix("_SET"))
            .and_then(|s| s.parse::<u32>().ok())
        {
            if !value.is_finite() {
                return Err(Error::Number);
            }
            let key = format!("GENERAL ENG STARTER:{index}");
            if !self.values.contains_key(&('A', key.clone())) {
                return Err(Error::Unsupported("starter is unavailable".into()));
            }
            self.set_number(&key, "bool", f64::from(value != 0.0));
            return Ok(());
        }
        if let Some(index) = name
            .strip_prefix("TOGGLE_STARTER")
            .and_then(|s| s.parse::<u32>().ok())
        {
            let key = format!("GENERAL ENG STARTER:{index}");
            if !self.values.contains_key(&('A', key.clone())) {
                return Err(Error::Unsupported("starter is unavailable".into()));
            }
            return self.toggle(&key);
        }
        if let Some(rest) = name.strip_prefix("MAGNETO") {
            let (engine, action) = if let Some(rest) = rest.strip_prefix('_') {
                (None, rest)
            } else if let Some((index, action)) = rest.split_once('_') {
                (
                    Some(
                        index
                            .parse::<u32>()
                            .map_err(|_| Error::Unsupported(format!("event {name}")))?,
                    ),
                    action,
                )
            } else {
                return Err(Error::Unsupported(format!("event {name}")));
            };
            return self.magneto_event(engine, action, value);
        }
        match name {
            "THROTTLE_FULL" | "THROTTLE_CUT" | "THROTTLE_INCR" | "THROTTLE_DECR"
            | "THROTTLE_SET" | "AXIS_THROTTLE_SET" => self.throttle_event(name, value, None),
            "THROTTLE1_SET" => self.throttle_event("THROTTLE_SET", value, Some(1)),
            "THROTTLE2_SET" => self.throttle_event("THROTTLE_SET", value, Some(2)),
            "THROTTLE3_SET" => self.throttle_event("THROTTLE_SET", value, Some(3)),
            "THROTTLE4_SET" => self.throttle_event("THROTTLE_SET", value, Some(4)),
            "AXIS_THROTTLE1_SET" => self.throttle_event("AXIS_THROTTLE_SET", value, Some(1)),
            "AXIS_THROTTLE2_SET" => self.throttle_event("AXIS_THROTTLE_SET", value, Some(2)),
            "AXIS_THROTTLE3_SET" => self.throttle_event("AXIS_THROTTLE_SET", value, Some(3)),
            "AXIS_THROTTLE4_SET" => self.throttle_event("AXIS_THROTTLE_SET", value, Some(4)),
            "MIXTURE_SET" | "MIXTURE_RICH" | "MIXTURE_LEAN" | "MIXTURE_INCR"
            | "MIXTURE_INCR_SMALL" | "MIXTURE_DECR" | "MIXTURE_DECR_SMALL" => {
                self.mixture_event(name, value, None)
            }
            "AXIS_MIXTURE_SET" => self.mixture_event(name, value, Some(1)),
            "MIXTURE1_SET" | "MIXTURE2_SET" | "MIXTURE3_SET" | "MIXTURE4_SET" => self
                .mixture_event(
                    "MIXTURE_SET",
                    value,
                    name.as_bytes().get(7).map(|n| u32::from(n - b'0')),
                ),
            "AXIS_MIXTURE1_SET" | "AXIS_MIXTURE2_SET" | "AXIS_MIXTURE3_SET"
            | "AXIS_MIXTURE4_SET" => self.mixture_event(
                "AXIS_MIXTURE_SET",
                value,
                name.as_bytes().get(12).map(|n| u32::from(n - b'0')),
            ),
            "ENGINE_AUTO_SHUTDOWN" => {
                for index in self.engines.clone() {
                    self.set_number(&format!("GENERAL ENG FUEL VALVE:{index}"), "bool", 0.0);
                }
                Ok(())
            }
            "FLAPS_UP" | "FLAPS_DOWN" | "FLAPS_INCR" | "FLAPS_DECR" | "FLAPS_SET" => self
                .flaps
                .as_mut()
                .ok_or_else(|| Error::Unsupported("flap system is unavailable".into()))?
                .event(name, value),
            "TOGGLE_FLIGHT_DIRECTOR" => self.toggle("AUTOPILOT FLIGHT DIRECTOR ACTIVE"),
            "TOGGLE_MASTER_BATTERY" => self.toggle("ELECTRICAL MASTER BATTERY"),
            "TOGGLE_AVIONICS_MASTER" => self.toggle("AVIONICS MASTER SWITCH"),
            "AVIONICS_MASTER_SET" => {
                self.set_number("AVIONICS MASTER SWITCH", "bool", f64::from(value != 0.0));
                Ok(())
            }
            "TOGGLE_MASTER_ALTERNATOR" | "TOGGLE_MASTER_BATTERY_ALTERNATOR" => {
                if name == "TOGGLE_MASTER_BATTERY_ALTERNATOR" {
                    self.toggle("ELECTRICAL MASTER BATTERY")?;
                }
                let generators = self
                    .electrical
                    .as_ref()
                    .map(|e| e.engine_indices())
                    .unwrap_or_else(|| self.engines.clone());
                for index in generators {
                    self.toggle(&format!("GENERAL ENG GENERATOR SWITCH:{index}"))?;
                }
                Ok(())
            }
            "TOGGLE_ALTERNATOR1" | "TOGGLE_ALTERNATOR2" | "TOGGLE_ALTERNATOR3"
            | "TOGGLE_ALTERNATOR4" => {
                let index = name.chars().last().unwrap();
                self.toggle(&format!("GENERAL ENG GENERATOR SWITCH:{index}"))
            }
            "TOGGLE_BATTERY_BUS_CONTACT" => self.toggle("ELECTRICAL BATTERY BUS CONTACT SWITCH"),
            "BATTERY_BUS_CONTACT_SET" => {
                self.set_number(
                    "ELECTRICAL BATTERY BUS CONTACT SWITCH",
                    "bool",
                    f64::from(value != 0.0),
                );
                Ok(())
            }
            "GENALT_BUS1_CONTACT_SET"
            | "GENALT_BUS2_CONTACT_SET"
            | "GENALT_BUS3_CONTACT_SET"
            | "GENALT_BUS4_CONTACT_SET" => {
                let index = name.as_bytes()[10] as char;
                self.set_number(
                    &format!("ELECTRICAL GENALT BUS CONTACT SWITCH:{index}"),
                    "bool",
                    f64::from(value != 0.0),
                );
                Ok(())
            }
            "TOGGLE_GENALT_BUS1_CONTACT"
            | "TOGGLE_GENALT_BUS2_CONTACT"
            | "TOGGLE_GENALT_BUS3_CONTACT"
            | "TOGGLE_GENALT_BUS4_CONTACT" => {
                let index = name.as_bytes()[17] as char;
                self.toggle(&format!("ELECTRICAL GENALT BUS CONTACT SWITCH:{index}"))
            }
            "TOGGLE_BEACON_LIGHTS" => self.toggle("LIGHT BEACON"),
            "TOGGLE_TAXI_LIGHTS" => self.toggle("LIGHT TAXI"),
            "TOGGLE_NAV_LIGHTS" => self.toggle("LIGHT NAV"),
            "STROBES_TOGGLE" => self.toggle("LIGHT STROBE"),
            "LANDING_LIGHTS_TOGGLE" => self.toggle("LIGHT LANDING"),
            "PITOT_HEAT_TOGGLE" => self.toggle("PITOT HEAT"),
            "PARKING_BRAKES" => self.toggle("BRAKE PARKING POSITION"),
            "GEAR_TOGGLE" => self.toggle("GEAR HANDLE POSITION"),
            "GEAR_UP" | "GEAR_DOWN" | "GEAR_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                self.set_number(
                    "GEAR HANDLE POSITION",
                    "bool",
                    f64::from(name == "GEAR_DOWN" || (name == "GEAR_SET" && value != 0.0)),
                );
                Ok(())
            }
            "ELEV_TRIM_UP" | "ELEV_TRIM_DN" | "ELEVATOR_TRIM_SET" | "AXIS_ELEV_TRIM_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let reference = Reference::new('A', "ELEVATOR TRIM PCT", Some("percent"));
                let current = self.read(&reference)?.number()?;
                let position = match name {
                    "ELEV_TRIM_UP" => current + 1.0,
                    "ELEV_TRIM_DN" => current - 1.0,
                    "AXIS_ELEV_TRIM_SET" => value / 16383.0 * 100.0,
                    _ => value / 16383.0 * 200.0 - 100.0,
                };
                self.set_number(
                    "ELEVATOR TRIM PCT",
                    "percent",
                    position.clamp(-100.0, 100.0),
                );
                Ok(())
            }
            "AXIS_LEFT_BRAKE_SET" | "AXIS_RIGHT_BRAKE_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let side = if name == "AXIS_LEFT_BRAKE_SET" {
                    "LEFT"
                } else {
                    "RIGHT"
                };
                self.set_number(
                    &format!("BRAKE {side} POSITION"),
                    "percent over 100",
                    ((value + 16383.0) / 32766.0).clamp(0.0, 1.0),
                );
                Ok(())
            }
            "SPOILERS_SET" | "AXIS_SPOILER_SET" | "SPOILERS_ON" | "SPOILERS_OFF"
            | "SPOILERS_TOGGLE" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let position = match name {
                    "SPOILERS_ON" => 100.0,
                    "SPOILERS_OFF" => 0.0,
                    "SPOILERS_TOGGLE" => {
                        let reference =
                            Reference::new('A', "SPOILERS HANDLE POSITION", Some("percent"));
                        if self
                            .read(&reference)
                            .ok()
                            .and_then(|v| v.number().ok())
                            .unwrap_or(0.0)
                            > 0.0
                        {
                            0.0
                        } else {
                            100.0
                        }
                    }
                    "AXIS_SPOILER_SET" => (value + 16383.0) * 100.0 / 32766.0,
                    _ => value * 100.0 / 16383.0,
                };
                self.set_number(
                    "SPOILERS HANDLE POSITION",
                    "percent",
                    position.clamp(0.0, 100.0),
                );
                Ok(())
            }
            "HEADING_BUG_SET" | "HEADING_BUG_INC" | "HEADING_BUG_DEC" => {
                let reference = Reference::new('A', "AUTOPILOT HEADING LOCK DIR", Some("degrees"));
                let heading = if name == "HEADING_BUG_SET" {
                    value
                } else {
                    let current = if self.values.contains_key(&('A', reference.name.clone())) {
                        self.read(&reference)?.number()?
                    } else {
                        0.0
                    };
                    current + if name == "HEADING_BUG_INC" { 1.0 } else { -1.0 }
                };
                self.set_number(
                    "AUTOPILOT HEADING LOCK DIR",
                    "degrees",
                    heading.rem_euclid(360.0),
                );
                Ok(())
            }
            _ => Err(Error::Unsupported(format!("event {name}"))),
        }
    }

    fn magneto_event(
        &mut self,
        engine: Option<u32>,
        action: &str,
        value: f64,
    ) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::Number);
        }
        let state = match action {
            "OFF" => 0,
            "RIGHT" => 1,
            "LEFT" => 2,
            "BOTH" | "START" => 3,
            "SET" | "SET_ACTUAL" if value.fract() == 0.0 && (0.0..=3.0).contains(&value) => {
                value as u8
            }
            _ => return Err(Error::Unsupported(format!("magneto action {action}"))),
        };
        let indices: Vec<_> = self
            .engines
            .iter()
            .copied()
            .filter(|i| engine.is_none_or(|e| e == *i))
            .collect();
        if indices.is_empty()
            || indices.iter().any(|i| {
                !self
                    .values
                    .contains_key(&('A', format!("RECIP ENG LEFT MAGNETO:{i}")))
            })
        {
            return Err(Error::Unsupported(
                "magneto controls are unavailable".into(),
            ));
        }
        for index in indices {
            let state = if engine.is_some() && matches!(action, "LEFT" | "RIGHT") {
                let left = self
                    .read(&Reference::new(
                        'A',
                        &format!("RECIP ENG LEFT MAGNETO:{index}"),
                        Some("bool"),
                    ))?
                    .number()?
                    != 0.0;
                let right = self
                    .read(&Reference::new(
                        'A',
                        &format!("RECIP ENG RIGHT MAGNETO:{index}"),
                        Some("bool"),
                    ))?
                    .number()?
                    != 0.0;
                (u8::from(right) | (u8::from(left) << 1)) ^ if action == "LEFT" { 2 } else { 1 }
            } else {
                state
            };
            self.set_number(
                &format!("RECIP ENG LEFT MAGNETO:{index}"),
                "bool",
                f64::from(state & 2 != 0),
            );
            self.set_number(
                &format!("RECIP ENG RIGHT MAGNETO:{index}"),
                "bool",
                f64::from(state & 1 != 0),
            );
            self.set_number(
                &format!("RECIP ENG MAGNETO:{index}"),
                "enum",
                f64::from(state),
            );
            if action == "START" {
                self.toggle(&format!("GENERAL ENG STARTER:{index}"))?;
            }
        }
        Ok(())
    }

    fn mixture_event(&mut self, name: &str, value: f64, engine: Option<u32>) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::Number);
        }
        let indices: Vec<_> = self
            .engines
            .iter()
            .copied()
            .filter(|i| engine.is_none_or(|e| e == *i))
            .collect();
        if indices.is_empty()
            || indices.iter().any(|i| {
                !self
                    .values
                    .contains_key(&('A', format!("GENERAL ENG MIXTURE LEVER POSITION:{i}")))
            })
        {
            return Err(Error::Unsupported(
                "mixture controls are unavailable".into(),
            ));
        }
        for index in indices {
            let variable = format!("GENERAL ENG MIXTURE LEVER POSITION:{index}");
            let current = self
                .read(&Reference::new('A', &variable, Some("percent")))?
                .number()?;
            let position = match name {
                "MIXTURE_RICH" => 100.0,
                "MIXTURE_LEAN" => 0.0,
                "MIXTURE_INCR" => current + 1.0,
                "MIXTURE_INCR_SMALL" => current + 0.1,
                "MIXTURE_DECR" => current - 1.0,
                "MIXTURE_DECR_SMALL" => current - 0.1,
                "AXIS_MIXTURE_SET" => (value + 16383.0) / 32766.0 * 100.0,
                _ => value / 16383.0 * 100.0,
            };
            self.set_number(&variable, "percent", position.clamp(0.0, 100.0));
        }
        Ok(())
    }

    fn throttle_event(&mut self, name: &str, value: f64, engine: Option<u32>) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::Number);
        }
        if self.engines.is_empty() || engine.is_some_and(|i| !self.engines.contains(&i)) {
            return Err(Error::Unsupported("engine controls are unavailable".into()));
        }
        for index in self
            .engines
            .clone()
            .into_iter()
            .filter(|i| engine.is_none_or(|e| e == *i))
        {
            let variable = format!("GENERAL ENG THROTTLE LEVER POSITION:{index}");
            let reference = Reference::new('A', &variable, Some("percent"));
            let position = match name {
                "THROTTLE_FULL" => 100.0,
                "THROTTLE_CUT" => 0.0,
                "THROTTLE_INCR" => self.read(&reference)?.number()? + 1.0,
                "THROTTLE_DECR" => self.read(&reference)?.number()? - 1.0,
                "AXIS_THROTTLE_SET" => (value + 16383.0) / 32766.0 * 100.0,
                _ => value / 16383.0 * 100.0,
            };
            self.set_number(&variable, "percent", position.clamp(0.0, 100.0));
        }
        Ok(())
    }
}

impl Host for Variables {
    fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
        let local = reference.category == 'L' || reference.category == 'G';
        let Some(variable) = self.values.get(&(
            reference.category,
            variable_name(reference.category, &reference.name),
        )) else {
            if reference.category == 'A'
                && let Some((value, unit)) = self
                    .flaps
                    .as_ref()
                    .and_then(|flaps| flaps.read(&reference.name))
            {
                let value = Value::Number(value);
                if reference.unit.as_deref() == Some(unit) {
                    return Ok(value);
                }
                return super::units::convert(&value, Some(unit), reference.unit.as_deref())
                    .ok_or_else(|| {
                        Error::Variable(format!(
                            "unsupported flap unit {}",
                            reference.unit.as_deref().unwrap_or("unitless")
                        ))
                    });
            }
            if local {
                return Ok(Value::Number(0.0));
            }
            return Err(Error::Variable(format!(
                "{}:{}, {}",
                reference.category,
                reference.name,
                reference.unit.as_deref().unwrap_or("")
            )));
        };
        if local || variable.unit == reference.unit {
            return Ok(variable.value.clone());
        }
        super::units::convert(
            &variable.value,
            variable.unit.as_deref(),
            reference.unit.as_deref(),
        )
        .ok_or_else(|| {
            Error::Variable(format!(
                "{}:{} cannot convert {} to {}",
                reference.category,
                reference.name,
                variable.unit.as_deref().unwrap_or("unitless"),
                reference.unit.as_deref().unwrap_or("unitless")
            ))
        })
    }
    fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error> {
        if reference.category == 'L' || reference.category == 'G' {
            self.set(
                reference.category,
                &reference.name,
                reference.unit.as_deref(),
                value,
            );
            return Ok(());
        }
        if reference.category == 'K' {
            self.events.push((reference.name.clone(), value));
            return Ok(());
        }
        Err(Error::Variable(format!(
            "{}:{} is not writable",
            reference.category, reference.name
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexed_magnetos_preserve_other_engines_and_report_the_switch_position() {
        let mut variables = Variables::default();
        variables.configure_engines([1, 2].into_iter()).unwrap();
        for index in [1, 2] {
            for side in ["LEFT", "RIGHT"] {
                variables.set_number(&format!("RECIP ENG {side} MAGNETO:{index}"), "bool", 1.0);
            }
        }
        variables
            .dispatch_key_event(0x1018e, Value::Number(0.0))
            .unwrap();
        let read = |variables: &mut Variables, name, unit| {
            variables
                .read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        assert_eq!(
            read(&mut variables, "RECIP ENG RIGHT MAGNETO:2", "bool"),
            0.0
        );
        assert_eq!(
            read(&mut variables, "RECIP ENG LEFT MAGNETO:2", "bool"),
            1.0
        );
        assert_eq!(
            read(&mut variables, "RECIP ENG LEFT MAGNETO:1", "bool"),
            1.0
        );
        assert_eq!(read(&mut variables, "RECIP ENG MAGNETO:2", "enum"), 2.0);
        assert!(
            variables
                .dispatch_event("MAGNETO2_SET", Value::Number(4.0))
                .is_err()
        );
        assert_eq!(read(&mut variables, "RECIP ENG MAGNETO:2", "enum"), 2.0);
    }

    #[test]
    fn mixture_events_control_indexed_levers_and_reject_invalid_values() {
        let mut variables = Variables::default();
        variables.configure_engines([1, 2].into_iter()).unwrap();
        for index in [1, 2] {
            variables.set_number(
                &format!("GENERAL ENG MIXTURE LEVER POSITION:{index}"),
                "percent",
                100.0,
            );
        }
        variables
            .dispatch_key_event(0x10180, Value::Number(8191.5))
            .unwrap();
        let read = |variables: &mut Variables, index| {
            variables
                .read(&Reference::new(
                    'A',
                    &format!("GENERAL ENG MIXTURE LEVER POSITION:{index}"),
                    Some("percent"),
                ))
                .unwrap()
                .number()
                .unwrap()
        };
        assert_eq!(read(&mut variables, 1), 100.0);
        assert_eq!(read(&mut variables, 2), 50.0);
        variables
            .dispatch_event("AXIS_MIXTURE1_SET", Value::Number(-16383.0))
            .unwrap();
        assert_eq!(read(&mut variables, 1), 0.0);
        assert!(
            variables
                .dispatch_event("MIXTURE_SET", Value::Number(f64::NAN))
                .is_err()
        );
        assert_eq!(read(&mut variables, 2), 50.0);
        variables
            .dispatch_event("MIXTURE_RICH", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, 1), 100.0);
        assert_eq!(read(&mut variables, 2), 100.0);
    }

    #[test]
    fn legacy_engine_names_share_canonical_indexed_values() {
        let mut variables = Variables::default();
        variables.set_number("TURB ENG N1:2", "percent", 82.5);
        assert_eq!(
            variables
                .read(&Reference::new('A', "Turb eng2 N1", Some("percent")))
                .unwrap()
                .number()
                .unwrap(),
            82.5
        );
        variables.set_number("GENERAL ENG1 THROTTLE LEVER POSITION", "percent", 35.0);
        let throttle = variables
            .read(&Reference::new(
                'A',
                "GENERAL ENG THROTTLE LEVER POSITION:1",
                Some("percent over 100"),
            ))
            .unwrap()
            .number()
            .unwrap();
        assert!((throttle - 0.35).abs() < 1e-12);
        variables.set('L', "TURB ENG1 N1", None, Value::Number(7.0));
        assert_eq!(
            variables
                .read(&Reference::new('L', "TURB ENG1 N1", None))
                .unwrap()
                .number()
                .unwrap(),
            7.0
        );
        assert!(
            variables
                .read(&Reference::new('A', "TURB ENG0 N1", Some("percent")))
                .is_err()
        );
    }

    #[test]
    fn throttle_events_share_indexed_engine_state_and_preserve_other_engines() {
        let mut variables = Variables::default();
        variables.configure_engines([1, 2].into_iter()).unwrap();
        let read = |variables: &mut Variables, index| {
            variables
                .read(&Reference::new(
                    'A',
                    &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
                    Some("percent"),
                ))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, 1), 100.0);
        assert_eq!(read(&mut variables, 2), 100.0);
        variables
            .dispatch_event("AXIS_THROTTLE1_SET", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, 1), 50.0);
        assert_eq!(read(&mut variables, 2), 100.0);
        variables
            .dispatch_event("THROTTLE2_SET", Value::Number(4095.75))
            .unwrap();
        assert_eq!(read(&mut variables, 2), 25.0);
        assert!(
            variables
                .dispatch_event("THROTTLE_SET", Value::Number(f64::NAN))
                .is_err()
        );
        assert_eq!(read(&mut variables, 2), 25.0);
        variables
            .dispatch_event("ENGINE_AUTO_SHUTDOWN", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            variables
                .read(&Reference::new(
                    'A',
                    "GENERAL ENG FUEL VALVE:1",
                    Some("bool")
                ))
                .unwrap()
                .number()
                .unwrap(),
            0.0
        );
        variables.set_number("TURB ENG JET THRUST:1", "newtons", 4.4482216152605 * 1000.0);
        assert!(
            (variables
                .read(&Reference::new(
                    'A',
                    "TURB ENG JET THRUST:1",
                    Some("pounds")
                ))
                .unwrap()
                .number()
                .unwrap()
                - 1000.0)
                .abs()
                < 1e-9
        );
    }
    use esp_script::Program;

    #[test]
    fn flap_actuators_use_airspeed_units_and_publish_surface_angles() {
        let config = esp_formats::cfg::Config::parse(
            "[flaps.0]\ntype=1\nextending-time=10\nflaps-position.0=0\nflaps-position.1=30,120",
        )
        .unwrap();
        let mut variables = Variables::default();
        variables
            .configure_flaps(&esp_formats::cfg::aircraft::flaps(&config).unwrap())
            .unwrap();
        variables
            .dispatch_event("FLAPS_DOWN", Value::Number(0.0))
            .unwrap();
        let angle = Reference::new('A', "TRAILING EDGE FLAPS0 LEFT ANGLE", Some("radians"));
        variables.set_number("AIRSPEED INDICATED", "meters per second", 70.0);
        variables.advance(10.0).unwrap();
        assert_eq!(variables.read(&angle).unwrap(), Value::Number(0.0));
        variables.set_number("AIRSPEED INDICATED", "meters per second", 50.0);
        variables.advance(10.0).unwrap();
        assert!(
            (variables.read(&angle).unwrap().number().unwrap() - std::f64::consts::PI / 6.0).abs()
                < 1e-12
        );
        variables.set_number("AIRSPEED INDICATED", "knots", f64::NAN);
        assert!(variables.advance(1.0).is_err());
    }

    #[test]
    fn shares_values_across_compatible_units() {
        let mut variables = Variables::default();
        let read = |variables: &mut Variables, unit| {
            variables
                .read(&Reference::new('A', "BRAKE PARKING POSITION", Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables.set_number("BRAKE PARKING POSITION", "percent", 100.0);
        assert_eq!(read(&mut variables, "bool"), 1.0);
        assert_eq!(read(&mut variables, "percent over 100"), 1.0);
        variables
            .dispatch_key_event(0x100d8, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "percent"), 0.0);
        variables
            .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "percent"), 100.0);
        assert_eq!(read(&mut variables, "boolean"), 1.0);
        variables.set_number(
            "AUTOPILOT HEADING LOCK DIR",
            "radians",
            std::f64::consts::PI,
        );
        variables
            .dispatch_event("HEADING_BUG_INC", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            variables
                .read(&Reference::new(
                    'A',
                    "AUTOPILOT HEADING LOCK DIR",
                    Some("degrees")
                ))
                .unwrap(),
            Value::Number(181.0)
        );
        assert!(
            variables
                .read(&Reference::new(
                    'A',
                    "BRAKE PARKING POSITION",
                    Some("degrees")
                ))
                .is_err()
        );
        assert!(
            variables
                .read(&Reference::new(
                    'A',
                    "BRAKE PARKING POSITION",
                    Some("unknown")
                ))
                .is_err()
        );
    }

    #[test]
    fn numeric_and_script_events_share_switch_state() {
        for (id, event, name) in [
            (
                0x102c1,
                "TOGGLE_MASTER_BATTERY",
                "ELECTRICAL MASTER BATTERY",
            ),
            (0x102f5, "TOGGLE_AVIONICS_MASTER", "AVIONICS MASTER SWITCH"),
            (0x102bf, "TOGGLE_BEACON_LIGHTS", "LIGHT BEACON"),
            (0x102c0, "TOGGLE_TAXI_LIGHTS", "LIGHT TAXI"),
            (0x1034b, "TOGGLE_NAV_LIGHTS", "LIGHT NAV"),
            (0x10018, "STROBES_TOGGLE", "LIGHT STROBE"),
            (0x100d7, "LANDING_LIGHTS_TOGGLE", "LIGHT LANDING"),
            (0x10142, "PITOT_HEAT_TOGGLE", "PITOT HEAT"),
            (0x100d8, "PARKING_BRAKES", "BRAKE PARKING POSITION"),
            (
                0x102f0,
                "TOGGLE_FLIGHT_DIRECTOR",
                "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
            ),
        ] {
            let mut variables = Variables::default();
            let reference = Reference::new('A', name, Some("bool"));
            variables
                .dispatch_key_event(id, Value::Number(0.0))
                .unwrap();
            assert_eq!(variables.read(&reference).unwrap(), Value::Number(1.0));
            variables.dispatch_event(event, Value::Number(0.0)).unwrap();
            assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
            assert!(
                variables
                    .dispatch_key_event(id, Value::Number(f64::NAN))
                    .is_err()
            );
            assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
        }
        assert!(
            Variables::default()
                .dispatch_key_event(u32::MAX, Value::Number(0.0))
                .is_err()
        );
    }

    #[test]
    fn separates_local_and_gauge_register_namespaces() {
        let mut variables = Variables::default();
        Program::compile("4 (>G:Var1) 8 (>L:Var1)")
            .unwrap()
            .evaluate(&mut variables)
            .unwrap();
        assert_eq!(
            variables
                .read(&Reference::new('G', "Var1", Some("number")))
                .unwrap(),
            Value::Number(4.0)
        );
        assert_eq!(
            variables.read(&Reference::new('L', "Var1", None)).unwrap(),
            Value::Number(8.0)
        );
        assert_eq!(
            variables.read(&Reference::new('G', "Var2", None)).unwrap(),
            Value::Number(0.0)
        );
        assert!(
            variables
                .read(&Reference::new('A', "MISSING", Some("bool")))
                .is_err()
        );
    }

    #[test]
    fn dispatches_queued_flight_director_events() {
        let mut variables = Variables::default();
        Program::compile("(>K:TOGGLE_FLIGHT_DIRECTOR)")
            .unwrap()
            .evaluate(&mut variables)
            .unwrap();
        let events = variables.drain_events().collect::<Vec<_>>();
        assert_eq!(
            events,
            [("TOGGLE_FLIGHT_DIRECTOR".into(), Value::Number(0.0))]
        );
        for (name, value) in events {
            variables.dispatch_event(&name, value).unwrap();
        }
        let reference = Reference::new('A', "AUTOPILOT FLIGHT DIRECTOR ACTIVE", Some("bool"));
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(1.0));
        variables
            .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Number(0.0))
            .unwrap();
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
        assert_eq!(variables.drain_events().count(), 0);
        assert!(
            variables
                .dispatch_event("UNKNOWN", Value::Number(0.0))
                .is_err()
        );
        assert!(
            variables
                .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Number(f64::NAN))
                .is_err()
        );
        assert!(
            variables
                .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Text("1".into()))
                .is_err()
        );
        assert!(variables.write(&reference, Value::Number(1.0)).is_err());
    }

    #[test]
    fn processes_events_even_after_script_failure() {
        let mut variables = Variables::default();
        assert!(
            Program::compile("(>K:TOGGLE_FLIGHT_DIRECTOR) (A:MISSING,bool)")
                .unwrap()
                .evaluate(&mut variables)
                .is_err()
        );
        assert!(variables.dispatch_events().is_empty());
        assert_eq!(variables.drain_events().count(), 0);
        assert_eq!(
            variables
                .read(&Reference::new(
                    'A',
                    "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
                    Some("bool")
                ))
                .unwrap(),
            Value::Number(1.0)
        );
        assert!(variables.dispatch_events().is_empty());
    }

    #[test]
    fn spoiler_handle_events_are_bounded_and_finite() {
        let mut variables = Variables::default();
        let reference = Reference::new('A', "SPOILERS HANDLE POSITION", Some("percent"));
        for (input, expected) in [
            (-1.0, 0.0),
            (0.0, 0.0),
            (8191.5, 50.0),
            (16383.0, 100.0),
            (16384.0, 100.0),
        ] {
            variables
                .dispatch_event("SPOILERS_SET", Value::Number(input))
                .unwrap();
            assert_eq!(variables.read(&reference).unwrap(), Value::Number(expected));
        }
        assert!(
            variables
                .dispatch_event("SPOILERS_SET", Value::Number(f64::NAN))
                .is_err()
        );
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(100.0));
    }

    #[test]
    fn selected_heading_events_wrap_and_preserve_state_on_error() {
        let mut variables = Variables::default();
        let reference = Reference::new('A', "AUTOPILOT HEADING LOCK DIR", Some("degrees"));
        for (event, input, expected) in [
            ("HEADING_BUG_DEC", 0.0, 359.0),
            ("HEADING_BUG_INC", 0.0, 0.0),
            ("HEADING_BUG_SET", 725.0, 5.0),
            ("HEADING_BUG_SET", -10.0, 350.0),
            ("HEADING_BUG_INC", 1.0, 351.0),
        ] {
            variables
                .dispatch_event(event, Value::Number(input))
                .unwrap();
            assert_eq!(variables.read(&reference).unwrap(), Value::Number(expected));
        }
        assert!(
            variables
                .dispatch_event("HEADING_BUG_SET", Value::Number(f64::INFINITY))
                .is_err()
        );
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(351.0));
    }
}
