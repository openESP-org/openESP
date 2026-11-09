use super::*;

impl Variables {
    pub fn dispatch_event(&mut self, name: &str, value: Value) -> Result<(), Error> {
        let value = value.number()?;
        let step = match name {
            "ELEV_DOWN" | "ELEVATOR_DOWN" => {
                Some(("ELEVATOR DEFLECTION PCT", "ELEVATOR_SET", -0.05))
            }
            "ELEV_UP" | "ELEVATOR_UP" => Some(("ELEVATOR DEFLECTION PCT", "ELEVATOR_SET", 0.05)),
            "AILERONS_LEFT" | "AILERON_LEFT" => Some((
                "AILERON LEFT DEFLECTION PCT",
                "AILERON_SET",
                -self.aileron_step_or_default(),
            )),
            "AILERONS_RIGHT" | "AILERON_RIGHT" => Some((
                "AILERON LEFT DEFLECTION PCT",
                "AILERON_SET",
                self.aileron_step_or_default(),
            )),
            "RUDDER_LEFT" => Some(("RUDDER DEFLECTION PCT", "RUDDER_SET", -0.05)),
            "RUDDER_RIGHT" => Some(("RUDDER DEFLECTION PCT", "RUDDER_SET", 0.05)),
            _ => None,
        };
        if let Some((input, event, increment)) = step {
            if !value.is_finite() {
                return Err(Error::Number);
            }
            let current = if self.has_control_input(input) {
                self.control_input(input)?
            } else {
                0.0
            };
            return self.dispatch_event(event, Value::Number((current + increment) * 16383.0));
        }
        if name == "RUDDER_CENTER" {
            return self.dispatch_event(
                "RUDDER_SET",
                Value::Number(if value.is_finite() { 0.0 } else { value }),
            );
        }
        if name == "AILERON_CENTER" {
            return self.dispatch_event("CENTER_AILER_RUDDER", Value::Number(value));
        }
        let controls: &[(&str, f64)] = match name {
            "ELEVATOR_SET" | "AXIS_ELEVATOR_SET" => {
                &[("YOKE Y POSITION", 1.0), ("ELEVATOR DEFLECTION PCT", 1.0)]
            }
            "AILERON_SET" | "AXIS_AILERONS_SET" => &[
                ("YOKE X POSITION", 1.0),
                ("AILERON LEFT DEFLECTION PCT", 1.0),
                ("AILERON RIGHT DEFLECTION PCT", -1.0),
            ],
            "RUDDER_SET" | "AXIS_RUDDER_SET" => &[
                ("RUDDER PEDAL POSITION", 1.0),
                ("RUDDER DEFLECTION PCT", 1.0),
            ],
            "CENTER_AILER_RUDDER" => &[
                ("YOKE X POSITION", 0.0),
                ("AILERON LEFT DEFLECTION PCT", 0.0),
                ("AILERON RIGHT DEFLECTION PCT", 0.0),
                ("RUDDER PEDAL POSITION", 0.0),
                ("RUDDER DEFLECTION PCT", 0.0),
            ],
            _ => &[],
        };
        if !controls.is_empty() {
            if !value.is_finite() {
                return Err(Error::Number);
            }
            let position = value.clamp(-16383.0, 16383.0) / 16383.0;
            for &(name, direction) in controls {
                self.set_number(name, "percent over 100", position * direction);
            }
            return Ok(());
        }
        if name == "TOGGLE_ELECT_FUEL_PUMP" || name == "FUEL_PUMP" {
            if self.engines.is_empty() {
                return Err(Error::Unsupported("fuel pumps are unavailable".into()));
            }
            for index in self.engines.clone() {
                self.toggle(&format!("GENERAL ENG FUEL PUMP SWITCH:{index}"))?;
            }
            return Ok(());
        }
        if let Some(index) = name
            .strip_prefix("TOGGLE_ELECT_FUEL_PUMP")
            .and_then(|index| index.parse::<u32>().ok())
        {
            if !self.engines.contains(&index) {
                return Err(Error::Unsupported(format!(
                    "fuel pump {index} is unavailable"
                )));
            }
            return self.toggle(&format!("GENERAL ENG FUEL PUMP SWITCH:{index}"));
        }
        if name == "TOGGLE_FUEL_VALVE_ALL" {
            if self.engines.is_empty() {
                return Err(Error::Unsupported("fuel valves are unavailable".into()));
            }
            for index in self.engines.clone() {
                self.toggle(&format!("GENERAL ENG FUEL VALVE:{index}"))?;
            }
            return Ok(());
        }
        if let Some(index) = name
            .strip_prefix("TOGGLE_FUEL_VALVE_ENG")
            .and_then(|index| index.parse::<u32>().ok())
        {
            if !self.engines.contains(&index) {
                return Err(Error::Unsupported(format!(
                    "fuel valve {index} is unavailable"
                )));
            }
            return self.toggle(&format!("GENERAL ENG FUEL VALVE:{index}"));
        }
        if let Some(rest) = name.strip_prefix("FUEL_SELECTOR_") {
            let (index, action) = if let Some((index, action)) = rest.split_once('_') {
                if let Ok(index) = index.parse::<u32>() {
                    (index, action)
                } else {
                    (1, rest)
                }
            } else {
                (1, rest)
            };
            let selector = match action {
                "OFF" => 0,
                "ALL" => 1,
                "LEFT" | "LEFT_MAIN" => 2,
                "RIGHT" | "RIGHT_MAIN" => 3,
                "LEFT_AUX" => 4,
                "RIGHT_AUX" => 5,
                "CENTER" => 6,
                "SET"
                    if value.is_finite()
                        && value.fract() == 0.0
                        && (0.0..=20.0).contains(&value) =>
                {
                    value as u32
                }
                _ => return Err(Error::Unsupported(format!("event {name}"))),
            };
            if (13..=15).contains(&selector) || selector == 18 {
                return Err(Error::Unsupported(format!(
                    "fuel selector position {selector}"
                )));
            }
            let key = format!("FUEL TANK SELECTOR:{index}");
            if !self.values.contains_key(&('A', key.clone())) {
                return Err(Error::Unsupported("fuel selector is unavailable".into()));
            }
            self.set_number(&key, "enum", f64::from(selector));
            return Ok(());
        }
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
            "TRUE_AIRSPEED_CAL_INC" | "TRUE_AIRSPEED_CAL_DEC" | "TRUE_AIRSPEED_CAL_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let current = self
                    .read(&Reference::new(
                        'A',
                        "AIRSPEED TRUE CALIBRATE",
                        Some("degrees"),
                    ))?
                    .number()?;
                let next = match name {
                    "TRUE_AIRSPEED_CAL_INC" => current + 1.0,
                    "TRUE_AIRSPEED_CAL_DEC" => current - 1.0,
                    _ => value,
                };
                self.set_number("AIRSPEED TRUE CALIBRATE", "degrees", next.rem_euclid(360.0));
                Ok(())
            }
            "KOHLSMAN_INC"
            | "KOHLSMAN_DEC"
            | "KOHLSMAN_SET"
            | "BAROMETRIC_STD_PRESSURE"
            | "BAROMETRIC" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let index = match name {
                    "KOHLSMAN_INC" | "KOHLSMAN_DEC" | "BAROMETRIC_STD_PRESSURE"
                        if value.fract() == 0.0 && (0.0..=4.0).contains(&value) =>
                    {
                        (value as u32).max(1)
                    }
                    "KOHLSMAN_INC" | "KOHLSMAN_DEC" | "BAROMETRIC_STD_PRESSURE" => {
                        return Err(Error::Number);
                    }
                    _ => 1,
                };
                let key = format!("KOHLSMAN SETTING MB:{index}");
                let current = self
                    .read(&Reference::new('A', &key, Some("pascals")))?
                    .number()?;
                let pressure = match name {
                    "KOHLSMAN_INC" => current + 33.863_895,
                    "KOHLSMAN_DEC" => current - 33.863_895,
                    "KOHLSMAN_SET" => value * 100.0 / 16.0,
                    "BAROMETRIC" => {
                        let altitude = self
                            .read(&Reference::new('A', "PLANE ALTITUDE", Some("feet")))
                            .ok()
                            .and_then(|value| value.number().ok())
                            .unwrap_or(0.0);
                        if altitude >= 18_000.0 {
                            101_325.0
                        } else {
                            self.read(&Reference::new('A', "SEA LEVEL PRESSURE", Some("pascals")))?
                                .number()?
                        }
                    }
                    _ => 101_325.0,
                };
                if !(80_000.0..=110_000.0).contains(&pressure) {
                    return Err(Error::Number);
                }
                self.set_number(&key, "pascals", pressure);
                Ok(())
            }
            "TOGGLE_VACUUM_FAILURE" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let current = self
                    .read(&Reference::new('A', "PARTIAL PANEL VACUUM", Some("enum")))?
                    .number()?;
                self.set_number("PARTIAL PANEL VACUUM", "enum", f64::from(current == 0.0));
                Ok(())
            }
            "TOGGLE_PITOT_BLOCKAGE" | "TOGGLE_STATIC_PORT_BLOCKAGE" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let target = if name == "TOGGLE_PITOT_BLOCKAGE" {
                    "PITOT BLOCKED"
                } else {
                    "STATIC PORT BLOCKED"
                };
                self.toggle(target)?;
                let mut blocked = false;
                for name in ["PITOT BLOCKED", "STATIC PORT BLOCKED"] {
                    blocked |= self
                        .read(&Reference::new('A', name, Some("bool")))?
                        .number()?
                        != 0.0;
                }
                self.set_number("PARTIAL PANEL PITOT", "enum", f64::from(blocked));
                Ok(())
            }
            "TOGGLE_MASTER_IGNITION_SWITCH" => self.toggle("MASTER IGNITION SWITCH"),
            "TURBINE_IGNITION_SWITCH_TOGGLE"
            | "TURBINE_IGNITION_SWITCH_SET"
            | "TURBINE_IGNITION_SWITCH_ON"
            | "TURBINE_IGNITION_SWITCH_OFF"
            | "TURBINE_IGNITION_SWITCH_AUTO" => {
                let reference = Reference::new('A', "TURB ENG IGNITION SWITCH", Some("enum"));
                let current = self.read(&reference)?.number()?;
                let mode = match name {
                    "TURBINE_IGNITION_SWITCH_TOGGLE" => {
                        if current == 0.0 {
                            1.0
                        } else {
                            0.0
                        }
                    }
                    "TURBINE_IGNITION_SWITCH_ON" => 2.0,
                    "TURBINE_IGNITION_SWITCH_OFF" => 0.0,
                    "TURBINE_IGNITION_SWITCH_AUTO" => 1.0,
                    _ if value.is_finite()
                        && value.fract() == 0.0
                        && (0.0..=2.0).contains(&value) =>
                    {
                        value
                    }
                    _ => return Err(Error::Number),
                };
                self.set_number("TURB ENG IGNITION SWITCH", "enum", mode);
                Ok(())
            }
            "THROTTLE_FULL" | "THROTTLE_CUT" | "THROTTLE_INCR" | "THROTTLE_DECR"
            | "THROTTLE_SET" | "AXIS_THROTTLE_SET" => self.throttle_event(name, value, None),
            "AXIS_THROTTLE_REVERSER_SET" => self.throttle_event(name, value, None),
            "AXIS_THROTTLE_REVERSER1_SET"
            | "AXIS_THROTTLE_REVERSER2_SET"
            | "AXIS_THROTTLE_REVERSER3_SET"
            | "AXIS_THROTTLE_REVERSER4_SET" => self.throttle_event(
                "AXIS_THROTTLE_REVERSER_SET",
                value,
                name.as_bytes().get(22).map(|index| u32::from(index - b'0')),
            ),
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
            "FLAPS_UP" | "FLAPS_1" | "FLAPS_2" | "FLAPS_3" | "FLAPS_4" | "FLAPS_DOWN"
            | "FLAPS_INCR" | "FLAPS_DECR" | "FLAPS_SET" | "AXIS_FLAPS_SET" => self
                .flaps
                .as_mut()
                .ok_or_else(|| Error::Unsupported("flap system is unavailable".into()))?
                .event(name, value),
            "TOGGLE_FLIGHT_DIRECTOR" => self.toggle("AUTOPILOT FLIGHT DIRECTOR ACTIVE"),
            "AP_MASTER" | "TOGGLE_AUTOPILOT" => {
                if !self.toggle_boolean("AUTOPILOT MASTER")? {
                    self.disable_autopilot_modes();
                }
                Ok(())
            }
            "AUTOPILOT_ON" => {
                self.set_boolean("AUTOPILOT MASTER", true);
                Ok(())
            }
            "AUTOPILOT_OFF" | "AUTOPILOT_DISENGAGE" => {
                self.set_boolean("AUTOPILOT MASTER", false);
                self.disable_autopilot_modes();
                Ok(())
            }
            "AP_PANEL_HEADING_HOLD" | "AP_HDG_HOLD" | "AP_HEADING_HOLD" => {
                if self.toggle_boolean("AUTOPILOT HEADING LOCK")? {
                    self.set_boolean("AUTOPILOT WING LEVELER", false);
                }
                Ok(())
            }
            "AP_WING_LEVELER" | "WING_LEVELER" => {
                if self.toggle_boolean("AUTOPILOT WING LEVELER")? {
                    self.set_boolean("AUTOPILOT HEADING LOCK", false);
                }
                Ok(())
            }
            "AP_PANEL_ALTITUDE_HOLD" | "AP_ALT_HOLD" => {
                if self.toggle_boolean("AUTOPILOT ALTITUDE LOCK")? {
                    self.set_boolean("AUTOPILOT VERTICAL HOLD", false);
                }
                Ok(())
            }
            "AP_PANEL_VS_HOLD" | "AP_VS_HOLD" => {
                if self.toggle_boolean("AUTOPILOT VERTICAL HOLD")? {
                    self.set_boolean("AUTOPILOT ALTITUDE LOCK", false);
                }
                Ok(())
            }
            "AP_MAX_BANK_INC" | "AP_MAX_BANK_DEC" | "AP_MAX_BANK_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let reference = Reference::new('A', "AUTOPILOT MAX BANK", Some("degrees"));
                let current = if self.values.contains_key(&('A', reference.name.clone())) {
                    self.read(&reference)?.number()?
                } else {
                    25.0
                };
                let bank = match name {
                    "AP_MAX_BANK_INC" => current + 5.0,
                    "AP_MAX_BANK_DEC" => current - 5.0,
                    _ => value,
                };
                self.set_number("AUTOPILOT MAX BANK", "degrees", bank.clamp(10.0, 30.0));
                Ok(())
            }
            "AP_ALT_VAR_INC"
            | "AP_ALT_VAR_DEC"
            | "AP_ALT_VAR_SET"
            | "AP_ALT_VAR_SET_ENGLISH"
            | "AP_ALT_VAR_SET_METRIC" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let reference = Reference::new('A', "AUTOPILOT ALTITUDE LOCK VAR", Some("feet"));
                let current = if self.values.contains_key(&('A', reference.name.clone())) {
                    self.read(&reference)?.number()?
                } else {
                    0.0
                };
                let altitude = match name {
                    "AP_ALT_VAR_INC" => {
                        crate::autopilot::selected_altitude_increment(current, 100.0)
                            .map_err(|_| Error::Number)?
                    }
                    "AP_ALT_VAR_DEC" => {
                        crate::autopilot::selected_altitude_increment(current, -100.0)
                            .map_err(|_| Error::Number)?
                    }
                    "AP_ALT_VAR_SET_METRIC" => crate::autopilot::selected_altitude_meters(value)
                        .map_err(|_| Error::Number)?,
                    _ => crate::autopilot::selected_altitude_increment(current, value - current)
                        .map_err(|_| Error::Number)?,
                };
                self.set_number("AUTOPILOT ALTITUDE LOCK VAR", "feet", altitude);
                Ok(())
            }
            "AP_VS_VAR_INC"
            | "AP_VS_VAR_DEC"
            | "AP_VS_VAR_SET"
            | "AP_VS_VAR_SET_ENGLISH"
            | "AP_VS_VAR_SET_METRIC" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let reference =
                    Reference::new('A', "AUTOPILOT VERTICAL HOLD VAR", Some("feet per minute"));
                let current = if self.values.contains_key(&('A', reference.name.clone())) {
                    self.read(&reference)?.number()?
                } else {
                    0.0
                };
                let vertical_speed = match name {
                    "AP_VS_VAR_INC" => {
                        crate::autopilot::selected_vertical_speed_increment(current, 100.0)
                            .map_err(|_| Error::Number)?
                    }
                    "AP_VS_VAR_DEC" => {
                        crate::autopilot::selected_vertical_speed_increment(current, -100.0)
                            .map_err(|_| Error::Number)?
                    }
                    "AP_VS_VAR_SET_METRIC" => {
                        crate::autopilot::selected_vertical_speed_meters_per_second(value)
                            .map_err(|_| Error::Number)?
                    }
                    _ => crate::autopilot::selected_vertical_speed_increment(
                        current,
                        value - current,
                    )
                    .map_err(|_| Error::Number)?,
                };
                self.set_number(
                    "AUTOPILOT VERTICAL HOLD VAR",
                    "feet per minute",
                    vertical_speed,
                );
                Ok(())
            }
            "HYDRAULIC_SWITCH_TOGGLE" => self.toggle("HYDRAULIC SWITCH"),
            "TOGGLE_HYDRAULIC_FAILURE" => self.toggle("HYDRAULIC FAILURE"),
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
            "PANEL_LIGHTS_TOGGLE" => self.toggle("LIGHT PANEL"),
            "PANEL_LIGHTS_ON" | "PANEL_LIGHTS_OFF" | "PANEL_LIGHTS_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                self.set_number(
                    "LIGHT PANEL",
                    "bool",
                    f64::from(match name {
                        "PANEL_LIGHTS_ON" => true,
                        "PANEL_LIGHTS_OFF" => false,
                        _ => value != 0.0,
                    }),
                );
                Ok(())
            }
            "LANDING_LIGHTS_TOGGLE" => self.toggle("LIGHT LANDING"),
            "PITOT_HEAT_TOGGLE" => self.toggle("PITOT HEAT"),
            "TOGGLE_STRUCTURAL_DEICE" => self.toggle("STRUCTURAL DEICE SWITCH"),
            "TOGGLE_GPS_DRIVES_NAV1" => self.toggle("GPS DRIVES NAV1"),
            "VOR1_SET" | "VOR1_OBI_INC" | "VOR1_OBI_DEC" | "VOR1_OBI_FAST_INC"
            | "VOR1_OBI_FAST_DEC" | "VOR2_SET" | "VOR2_OBI_INC" | "VOR2_OBI_DEC"
            | "VOR2_OBI_FAST_INC" | "VOR2_OBI_FAST_DEC" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let index = if name.starts_with("VOR2") { 2 } else { 1 };
                let variable = format!("NAV OBS:{index}");
                let current = self
                    .read(&Reference::new('A', &variable, Some("degrees")))?
                    .number()?;
                let selected = if name.ends_with("_SET") {
                    value
                } else if name.ends_with("FAST_INC") {
                    (current + 10.0).min(360.0)
                } else if name.ends_with("FAST_DEC") {
                    (current - 10.0).max(0.0)
                } else if name.ends_with("_INC") {
                    current + 1.0
                } else {
                    current - 1.0
                };
                self.set_number(&variable, "degrees", selected.rem_euclid(360.0));
                Ok(())
            }
            "NAV1_RADIO_SET_HZ"
            | "NAV1_STBY_SET_HZ"
            | "NAV1_RADIO_SET"
            | "NAV1_STBY_SET"
            | "NAV1_RADIO_SWAP"
            | "NAV1_RADIO_WHOLE_INC"
            | "NAV1_RADIO_WHOLE_DEC"
            | "NAV1_RADIO_FRACT_INC"
            | "NAV1_RADIO_FRACT_DEC"
            | "NAV1_RADIO_FRACT_INC_CARRY"
            | "NAV1_RADIO_FRACT_DEC_CARRY"
            | "NAV2_RADIO_SET_HZ"
            | "NAV2_STBY_SET_HZ"
            | "NAV2_RADIO_SET"
            | "NAV2_STBY_SET"
            | "NAV2_RADIO_SWAP"
            | "NAV2_RADIO_WHOLE_INC"
            | "NAV2_RADIO_WHOLE_DEC"
            | "NAV2_RADIO_FRACT_INC"
            | "NAV2_RADIO_FRACT_DEC"
            | "NAV2_RADIO_FRACT_INC_CARRY"
            | "NAV2_RADIO_FRACT_DEC_CARRY" => self.nav_radio_event(name, value),
            "PARKING_BRAKES" => self.toggle("BRAKE PARKING POSITION"),
            "SPOILERS_ARM_TOGGLE" => self.toggle("SPOILERS ARMED"),
            "SPOILERS_ARM_ON" | "SPOILERS_ARM_OFF" | "SPOILERS_ARM_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                self.set_number(
                    "SPOILERS ARMED",
                    "bool",
                    f64::from(
                        name == "SPOILERS_ARM_ON" || (name == "SPOILERS_ARM_SET" && value != 0.0),
                    ),
                );
                Ok(())
            }
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
            "AILERON_TRIM_LEFT" | "AILERON_TRIM_RIGHT" | "AILERON_TRIM_SET"
            | "RUDDER_TRIM_LEFT" | "RUDDER_TRIM_RIGHT" | "RUDDER_TRIM_SET" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let variable = if name.starts_with("AILERON") {
                    "AILERON TRIM PCT"
                } else {
                    "RUDDER TRIM PCT"
                };
                let current = self
                    .read(&Reference::new('A', variable, Some("percent")))?
                    .number()?;
                let position = if name.ends_with("_LEFT") {
                    current - 1.0
                } else if name.ends_with("_RIGHT") {
                    current + 1.0
                } else {
                    value / 16383.0 * 200.0 - 100.0
                };
                self.set_number(variable, "percent", position.clamp(-100.0, 100.0));
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
            "HEADING_GYRO_SET" | "GYRO_DRIFT_SET" | "GYRO_DRIFT_INC" | "GYRO_DRIFT_DEC" => {
                if !value.is_finite() {
                    return Err(Error::Number);
                }
                let drift = match name {
                    "HEADING_GYRO_SET" => 0.0,
                    "GYRO_DRIFT_SET" => value,
                    _ => {
                        let current = self
                            .read(&Reference::new('A', "GYRO DRIFT ERROR", Some("degrees")))?
                            .number()?;
                        current + if name == "GYRO_DRIFT_INC" { 1.0 } else { -1.0 }
                    }
                };
                if !drift.is_finite() {
                    return Err(Error::Number);
                }
                self.set_number(
                    "GYRO DRIFT ERROR",
                    "degrees",
                    (drift + 180.0).rem_euclid(360.0) - 180.0,
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

    pub(super) fn throttle_event(
        &mut self,
        name: &str,
        value: f64,
        engine: Option<u32>,
    ) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::Number);
        }
        if self.engines.is_empty() || engine.is_some_and(|i| !self.engines.contains(&i)) {
            return Err(Error::Unsupported("engine controls are unavailable".into()));
        }
        if name == "AXIS_THROTTLE_REVERSER_SET" && self.throttle_lower_limit == 0.0 {
            return Err(Error::Unsupported(
                "thrust reversers are unavailable".into(),
            ));
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
                "AXIS_THROTTLE_REVERSER_SET" => {
                    (value.clamp(-16383.0, 16383.0) + 16383.0) / 32766.0
                        * self.throttle_lower_limit
                        * 100.0
                }
                _ => value / 16383.0 * 100.0,
            };
            self.set_number(
                &variable,
                "percent",
                position.clamp(self.throttle_lower_limit * 100.0, 100.0),
            );
        }
        Ok(())
    }
}
