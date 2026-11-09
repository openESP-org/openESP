use esp_script::{Error, Host, Reference, Value};
use std::collections::HashMap;

pub struct Variables {
    values: HashMap<(char, String), Variable>,
    events: Vec<(String, Value)>,
    flaps: Option<crate::flaps::Flaps>,
    engines: Vec<u32>,
    electrical: Option<crate::electrical::Electrical>,
    throttle_lower_limit: f64,
    control_surfaces: Option<[f64; 4]>,
    aileron_step: f64,
    air_data_pressures: Option<[f64; 2]>,
}

impl Default for Variables {
    fn default() -> Self {
        let mut variables = Self {
            values: HashMap::new(),
            events: Vec::new(),
            flaps: None,
            engines: Vec::new(),
            electrical: None,
            throttle_lower_limit: 0.0,
            control_surfaces: None,
            aileron_step: 0.0,
            air_data_pressures: None,
        };
        for name in [
            "ADF",
            "AIRSPEED",
            "ALTIMETER",
            "ATTITUDE",
            "COMM",
            "COMPASS",
            "ELECTRICAL",
            "AVIONICS",
            "ENGINE",
            "FUEL INDICATOR",
            "HEADING",
            "VERTICAL VELOCITY",
            "TRANSPONDER",
            "NAV",
            "PITOT",
            "TURN COORDINATOR",
            "VACUUM",
        ] {
            variables.set_number(&format!("PARTIAL PANEL {name}"), "enum", 0.0);
        }
        for name in ["PITOT BLOCKED", "STATIC PORT BLOCKED"] {
            variables.set_number(name, "bool", 0.0);
        }
        variables.set_number("TRUE AIRSPEED SELECTED", "bool", 0.0);
        variables.set_number("AIRSPEED TRUE CALIBRATE", "degrees", 0.0);
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", 0.0);
        variables.set_number("AMBIENT WIND VELOCITY", "knots", 0.0);
        variables.set_number("SEA LEVEL PRESSURE", "millibars", 1013.25);
        variables.set_number("SEA LEVEL TEMPERATURE", "celsius", 15.0);
        variables.set_number("WORLD ORIGIN LATITUDE", "degrees", 0.0);
        variables.set_number("WORLD ORIGIN LONGITUDE", "degrees", 0.0);
        variables.set_number("MAGVAR", "degrees", 0.0);
        variables.set_number("GYRO DRIFT ERROR", "degrees", 0.0);
        variables.set_number("TURN COORDINATOR BALL", "position 128", 0.0);
        variables.set_number("TURN COORDINATOR BALL INV", "position 128", 0.0);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        variables.set_number("GPS WP NEXT LAT", "degrees", 0.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 0.0);
        variables.set_number("GPS DIRECT LEG VALID", "bool", 0.0);
        variables.set_number("GPS CDI SCALING", "meters", 9260.0);
        variables.set_number("GPS DRIVES NAV1", "bool", 0.0);
        variables.set_number("HSI GPS SOURCE ACTIVE", "bool", 0.0);
        variables.set_number("VOR STATION ACTIVE", "bool", 0.0);
        variables.set_number("VOR STATION COUNT", "number", 1.0);
        variables.set_number("VOR STATION LATITUDE", "degrees", 0.0);
        variables.set_number("VOR STATION LONGITUDE", "degrees", 0.0);
        variables.set_number("VOR STATION FREQUENCY", "megahertz", 113.0);
        variables.set_number("VOR STATION RANGE", "nautical miles", 40.0);
        variables.set_number("VOR STATION LOCALIZER", "bool", 0.0);
        variables.set_number("ILS LOCALIZER COURSE", "degrees", 0.0);
        variables.set_number("ILS GLIDESLOPE ANGLE", "degrees", 3.0);
        variables.set_number("NAV ACTIVE FREQUENCY:1", "megahertz", 113.0);
        variables.set_number("NAV STANDBY FREQUENCY:1", "megahertz", 110.5);
        variables.set_number("NAV OBS:1", "degrees", 0.0);
        variables.set_number("NAV ACTIVE FREQUENCY:2", "megahertz", 113.0);
        variables.set_number("NAV STANDBY FREQUENCY:2", "megahertz", 110.5);
        variables.set_number("NAV OBS:2", "degrees", 0.0);
        for index in 1..=4 {
            variables.set_number(&format!("KOHLSMAN SETTING MB:{index}"), "pascals", 101325.0);
        }
        variables
    }
}

struct Variable {
    unit: Option<String>,
    value: Value,
}

fn variable_name(category: char, name: &str) -> String {
    if category == 'A' {
        for prefix in ["NAV", "NAV1", "NAV2"] {
            let index = if prefix == "NAV2" { 2 } else { 1 };
            for suffix in ["ACTIVE FREQUENCY", "STANDBY FREQUENCY"] {
                if name == format!("{prefix} {suffix}") {
                    return format!("NAV {suffix}:{index}");
                }
            }
        }
    }
    if category == 'A' {
        for prefix in [
            "KOHLSMAN SETTING MB",
            "KOHLSMAN SETTING HG",
            "KOHLSMAN SETTING",
        ] {
            if let Some(suffix) = name.strip_prefix(prefix) {
                if suffix.is_empty() {
                    return "KOHLSMAN SETTING:1".into();
                }
                if let Some(index) = suffix.strip_prefix(':')
                    && index.parse::<u32>().is_ok_and(|n| n > 0)
                {
                    return format!("KOHLSMAN SETTING:{index}");
                }
            }
        }
    }
    if category == 'A' && name == "FUEL TANK SELECTOR" {
        return "FUEL TANK SELECTOR:1".into();
    }
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
    pub fn air_data_pressures(
        &mut self,
        total: f64,
        ambient: f64,
        pitot_blocked: bool,
        static_blocked: bool,
    ) -> anyhow::Result<[f64; 2]> {
        anyhow::ensure!(
            total.is_finite() && ambient.is_finite() && total >= ambient && ambient > 0.0,
            "invalid air data pressures"
        );
        let previous = self.air_data_pressures.unwrap_or([total, ambient]);
        let current = [
            if pitot_blocked { previous[0] } else { total },
            if static_blocked { previous[1] } else { ambient },
        ];
        self.air_data_pressures = Some(current);
        Ok(current)
    }

    pub fn static_line_pressure(&self) -> Option<f64> {
        self.air_data_pressures.map(|pressures| pressures[1])
    }

    pub fn pitot_line_pressure(&self) -> Option<f64> {
        self.air_data_pressures.map(|pressures| pressures[0])
    }

    pub fn configure_aileron_travel(&mut self, up_limit_degrees: f64) -> anyhow::Result<()> {
        anyhow::ensure!(
            up_limit_degrees.is_finite() && up_limit_degrees >= 0.0,
            "invalid aileron travel"
        );
        self.aileron_step = if up_limit_degrees > 0.0 {
            (1.0 / up_limit_degrees).min(1.0)
        } else {
            0.0
        };
        Ok(())
    }

    fn aileron_step_or_default(&self) -> f64 {
        if self.aileron_step > 0.0 {
            self.aileron_step
        } else {
            0.05
        }
    }

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
            self.set_number(
                &format!("GENERAL ENG FUEL PUMP SWITCH:{index}"),
                "bool",
                0.0,
            );
        }
        self.engines = engines;
        self.configure_throttle_limit(0.0)?;
        Ok(())
    }

    pub fn configure_throttle_limit(&mut self, lower: f64) -> anyhow::Result<()> {
        anyhow::ensure!(
            lower.is_finite() && (-1.0..=0.0).contains(&lower),
            "invalid throttle lower limit"
        );
        self.throttle_lower_limit = lower;
        self.set_number("THROTTLE LOWER LIMIT", "percent", lower * 100.0);
        Ok(())
    }

    pub fn configure_fuel_selectors(&mut self, count: u32) {
        for index in 1..=count {
            self.set_number(&format!("FUEL TANK SELECTOR:{index}"), "enum", 1.0);
        }
    }

    pub fn control_input(&self, name: &str) -> Result<f64, Error> {
        let variable = self
            .values
            .get(&('A', name.to_owned()))
            .ok_or_else(|| Error::Variable(format!("A:{name}")))?;
        if variable.unit.as_deref() == Some("percent over 100") {
            return variable.value.number();
        }
        super::units::convert(
            &variable.value,
            variable.unit.as_deref(),
            Some("percent over 100"),
        )
        .ok_or_else(|| Error::Variable(format!("unsupported control unit for {name}")))?
        .number()
    }

    pub fn has_control_input(&self, name: &str) -> bool {
        self.values.contains_key(&('A', name.to_owned()))
    }

    pub fn set_control_surfaces(&mut self, surfaces: [f64; 4]) {
        self.control_surfaces = Some(surfaces);
    }

    pub fn flap_angles(&self) -> impl Iterator<Item = f64> + '_ {
        self.flaps.iter().flat_map(|flaps| flaps.angles())
    }

    pub fn flap_handle_fraction(&self) -> f64 {
        self.flaps
            .as_ref()
            .and_then(|flaps| flaps.read("FLAPS HANDLE PERCENT"))
            .map_or(0.0, |(value, _)| value)
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
        self.advance_flaps(dt, None)
    }

    pub fn advance_flaps(
        &mut self,
        dt: f64,
        hydraulic_fraction: Option<f64>,
    ) -> anyhow::Result<()> {
        anyhow::ensure!(dt.is_finite() && dt >= 0.0, "invalid simulation timestep");
        let reference = Reference::new('A', "AIRSPEED INDICATED", Some("knots"));
        let airspeed = if self.values.contains_key(&('A', reference.name.clone())) {
            self.read(&reference)?.number()?
        } else {
            0.0
        };
        let electric = if self.electrical.is_some() {
            self.read(&Reference::new('A', "CIRCUIT FLAP MOTOR ON", Some("bool")))?
                .number()?
        } else {
            1.0
        };
        if let Some(flaps) = &mut self.flaps {
            flaps.advance_with_power(
                dt,
                airspeed,
                crate::flaps::Power {
                    electric,
                    hydraulic: hydraulic_fraction.unwrap_or(1.0),
                },
            )?;
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
        if (0x109ce..=0x109d2).contains(&id) {
            let index = id - 0x109ce;
            return self.throttle_event(
                "AXIS_THROTTLE_REVERSER_SET",
                value.number()?,
                (index != 0).then_some(index),
            );
        }
        if (0x103bd..=0x103c1).contains(&id) {
            let offset = id - 0x103bd;
            let name = if offset == 0 {
                "TOGGLE_FUEL_VALVE_ALL".to_owned()
            } else {
                format!("TOGGLE_FUEL_VALVE_ENG{offset}")
            };
            return self.dispatch_event(&name, value);
        }
        for (start, selector) in [(0x101a3, 1), (0x103d4, 2), (0x10401, 3), (0x10409, 4)] {
            if (start..start + 8).contains(&id) {
                let action = [
                    "OFF",
                    "ALL",
                    "LEFT",
                    "RIGHT",
                    "LEFT_AUX",
                    "RIGHT_AUX",
                    "CENTER",
                    "SET",
                ][(id - start) as usize];
                return self.dispatch_event(&format!("FUEL_SELECTOR_{selector}_{action}"), value);
            }
        }
        if (0x1046d..=0x10474).contains(&id) {
            let offset = id - 0x1046d;
            let action = if offset < 4 {
                "LEFT_MAIN"
            } else {
                "RIGHT_MAIN"
            };
            return self
                .dispatch_event(&format!("FUEL_SELECTOR_{}_{action}", offset % 4 + 1), value);
        }
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
            0x10020 => "HEADING_GYRO_SET",
            0x10068 => "NAV1_RADIO_WHOLE_DEC",
            0x10069 => "NAV1_RADIO_WHOLE_INC",
            0x1006a => "NAV1_RADIO_FRACT_DEC",
            0x1006b => "NAV1_RADIO_FRACT_INC",
            0x1006c => "NAV2_RADIO_WHOLE_DEC",
            0x1006d => "NAV2_RADIO_WHOLE_INC",
            0x1006e => "NAV2_RADIO_FRACT_DEC",
            0x1006f => "NAV2_RADIO_FRACT_INC",
            0x1007e => "VOR1_OBI_DEC",
            0x1007f => "VOR1_OBI_INC",
            0x10080 => "VOR2_OBI_DEC",
            0x10081 => "VOR2_OBI_INC",
            0x100ac => "NAV1_RADIO_SET",
            0x100ad => "NAV2_RADIO_SET",
            0x100b4 => "VOR1_SET",
            0x100b5 => "VOR2_SET",
            0x10155 => "GYRO_DRIFT_INC",
            0x10156 => "GYRO_DRIFT_DEC",
            0x101fe => "GYRO_DRIFT_SET",
            0x1033f => "VOR1_OBI_FAST_DEC",
            0x10340 => "VOR1_OBI_FAST_INC",
            0x10341 => "VOR2_OBI_FAST_DEC",
            0x10342 => "VOR2_OBI_FAST_INC",
            0x10347 => "TOGGLE_GPS_DRIVES_NAV1",
            0x1038d => "NAV1_RADIO_FRACT_DEC_CARRY",
            0x1038e => "NAV1_RADIO_FRACT_INC_CARRY",
            0x1038f => "NAV1_STBY_SET",
            0x10390 => "NAV1_RADIO_SWAP",
            0x10391 => "NAV2_RADIO_FRACT_DEC_CARRY",
            0x10392 => "NAV2_RADIO_FRACT_INC_CARRY",
            0x10393 => "NAV2_STBY_SET",
            0x10394 => "NAV2_RADIO_SWAP",
            0x10018 => "STROBES_TOGGLE",
            0x10030 => "BAROMETRIC",
            0x10022 => "GEAR_TOGGLE",
            0x10035 => "SPOILERS_TOGGLE",
            0x1003b => "FLAPS_UP",
            0x1003d => "FLAPS_1",
            0x1003f => "FLAPS_2",
            0x10041 => "FLAPS_3",
            0x10043 => "FLAPS_DOWN",
            0x100a4 => "GEAR_SET",
            0x1021f => "GEAR_UP",
            0x10220 => "GEAR_DOWN",
            0x10047 => "ELEV_TRIM_DN",
            0x10048 => "ELEV_DOWN",
            0x1004b => "AILERONS_LEFT",
            0x1004c => "CENTER_AILER_RUDDER",
            0x1004d => "AILERONS_RIGHT",
            0x1004f => "ELEV_TRIM_UP",
            0x10050 => "ELEV_UP",
            0x10093 => "ELEVATOR_DOWN",
            0x10094 => "ELEVATOR_UP",
            0x10095 => "AILERON_LEFT",
            0x10096 => "AILERON_CENTER",
            0x10097 => "AILERON_RIGHT",
            0x10098 => "RUDDER_LEFT",
            0x10099 => "RUDDER_CENTER",
            0x1009a => "RUDDER_RIGHT",
            0x1009e => "ELEVATOR_SET",
            0x1009f => "AILERON_SET",
            0x100a0 => "RUDDER_SET",
            0x100e2 => "AXIS_ELEVATOR_SET",
            0x100e3 => "AXIS_AILERONS_SET",
            0x100e4 => "AXIS_RUDDER_SET",
            0x100aa => "ELEVATOR_TRIM_SET",
            0x100e6 => "AXIS_ELEV_TRIM_SET",
            0x100a2 => "FLAPS_SET",
            0x100de => "FLAPS_INCR",
            0x100df => "FLAPS_DECR",
            0x100d6 => "PANEL_LIGHTS_TOGGLE",
            0x100d7 => "LANDING_LIGHTS_TOGGLE",
            0x10208 => "PANEL_LIGHTS_ON",
            0x10209 => "PANEL_LIGHTS_OFF",
            0x1020a => "PANEL_LIGHTS_SET",
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
            0x1015b => "KOHLSMAN_INC",
            0x1015c => "KOHLSMAN_DEC",
            0x1015d => "TRUE_AIRSPEED_CAL_INC",
            0x1015e => "TRUE_AIRSPEED_CAL_DEC",
            0x101f8 => "KOHLSMAN_SET",
            0x101fb => "TRUE_AIRSPEED_CAL_SET",
            0x1051e => "BAROMETRIC_STD_PRESSURE",
            0x1013d => "SPOILERS_ARM_TOGGLE",
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
            0x10212 => "SPOILERS_ARM_ON",
            0x10213 => "SPOILERS_ARM_OFF",
            0x10214 => "SPOILERS_ARM_SET",
            0x10305 => "TOGGLE_HYDRAULIC_FAILURE",
            0x10301 => "TOGGLE_VACUUM_FAILURE",
            0x10303 => "TOGGLE_PITOT_BLOCKAGE",
            0x10304 => "TOGGLE_STATIC_PORT_BLOCKAGE",
            0x10498 => "HYDRAULIC_SWITCH_TOGGLE",
            0x103e7 => "TOGGLE_MASTER_IGNITION_SWITCH",
            0x1049d => "TURBINE_IGNITION_SWITCH_TOGGLE",
            0x10794 => "TURBINE_IGNITION_SWITCH_SET",
            0x10795 => "TURBINE_IGNITION_SWITCH_ON",
            0x10796 => "TURBINE_IGNITION_SWITCH_OFF",
            0x10797 => "TURBINE_IGNITION_SWITCH_AUTO",
            0x102c0 => "TOGGLE_TAXI_LIGHTS",
            0x102c1 => "TOGGLE_MASTER_BATTERY",
            0x102c2 => "TOGGLE_MASTER_ALTERNATOR",
            0x102f0 => "TOGGLE_FLIGHT_DIRECTOR",
            0x102e4 => "AILERON_TRIM_LEFT",
            0x102e5 => "AILERON_TRIM_RIGHT",
            0x102e6 => "RUDDER_TRIM_LEFT",
            0x102e7 => "RUDDER_TRIM_RIGHT",
            0x104ab => "AILERON_TRIM_SET",
            0x104ac => "RUDDER_TRIM_SET",
            0x102f4 => "AXIS_MIXTURE_SET",
            0x102f5 => "TOGGLE_AVIONICS_MASTER",
            0x102fc => "TOGGLE_STARTER1",
            0x102fd => "TOGGLE_STARTER2",
            0x102fe => "TOGGLE_STARTER3",
            0x102ff => "TOGGLE_STARTER4",
            0x1034b => "TOGGLE_NAV_LIGHTS",
            0x1034e => "AXIS_SPOILER_SET",
            0x103e6 => "AXIS_FLAPS_SET",
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

    fn boolean(&mut self, name: &str) -> Result<bool, Error> {
        let reference = Reference::new('A', name, Some("bool"));
        if self.values.contains_key(&('A', name.to_owned())) {
            Ok(self.read(&reference)?.number()? != 0.0)
        } else {
            Ok(false)
        }
    }

    fn set_boolean(&mut self, name: &str, active: bool) {
        self.set_number(name, "bool", f64::from(active));
    }

    fn toggle_boolean(&mut self, name: &str) -> Result<bool, Error> {
        let active = !self.boolean(name)?;
        self.set_boolean(name, active);
        Ok(active)
    }

    fn toggle(&mut self, name: &str) -> Result<(), Error> {
        self.toggle_boolean(name).map(|_| ())
    }

    fn disable_autopilot_modes(&mut self) {
        for name in [
            "AUTOPILOT HEADING LOCK",
            "AUTOPILOT ALTITUDE LOCK",
            "AUTOPILOT VERTICAL HOLD",
            "AUTOPILOT WING LEVELER",
        ] {
            self.set_boolean(name, false);
        }
    }

    fn nav_radio_event(&mut self, name: &str, value: f64) -> Result<(), Error> {
        let index = if name.starts_with("NAV2") { 2 } else { 1 };
        let name = name.replacen("NAV2", "NAV1", 1);
        let name = name.as_str();
        let active_name = format!("NAV ACTIVE FREQUENCY:{index}");
        let standby_name = format!("NAV STANDBY FREQUENCY:{index}");
        let active = active_name.as_str();
        let standby = standby_name.as_str();
        let read_hz = |v: &mut Self, name| -> Result<f64, Error> {
            let value = v
                .read(&Reference::new('A', name, Some("hertz")))?
                .number()?;
            if !value.is_finite() || !(108_000_000.0..118_000_000.0).contains(&value) {
                return Err(Error::Number);
            }
            Ok(value.round())
        };
        if name == "NAV1_RADIO_SWAP" {
            let active_value = read_hz(self, active)?;
            let standby_value = read_hz(self, standby)?;
            self.set_number(active, "hertz", standby_value);
            self.set_number(standby, "hertz", active_value);
            return Ok(());
        }
        if name.ends_with("SET") || name.ends_with("SET_HZ") {
            if !value.is_finite() {
                return Err(Error::Number);
            }
            let hz = if name.ends_with("SET_HZ") {
                value
            } else {
                crate::units::frequency_from_bcd16(value).ok_or(Error::Number)?
            };
            if !(108_000_000.0..118_000_000.0).contains(&hz) {
                return Err(Error::Number);
            }
            self.set_number(
                if name.starts_with("NAV1_STBY") {
                    standby
                } else {
                    active
                },
                "hertz",
                hz,
            );
            return Ok(());
        }
        let hz = read_hz(self, standby)? as i64;
        let increase = name.contains("_INC");
        let direction = if increase { 1 } else { -1 };
        let next = if name.contains("WHOLE") {
            108_000_000 + (hz - 108_000_000 + direction * 1_000_000).rem_euclid(10_000_000)
        } else if name.ends_with("CARRY") {
            108_000_000 + (hz - 108_000_000 + direction * 50_000).rem_euclid(10_000_000)
        } else {
            hz - hz.rem_euclid(1_000_000)
                + (hz.rem_euclid(1_000_000) + direction * 25_000).rem_euclid(1_000_000)
        };
        self.set_number(standby, "hertz", next as f64);
        Ok(())
    }
}

impl Host for Variables {
    fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
        if reference.category == 'A'
            && let Some(surfaces) = self.control_surfaces
        {
            let index = match reference.name.as_str() {
                "ELEVATOR DEFLECTION PCT" => Some(0),
                "AILERON LEFT DEFLECTION PCT" => Some(1),
                "AILERON RIGHT DEFLECTION PCT" => Some(2),
                "RUDDER DEFLECTION PCT" => Some(3),
                _ => None,
            };
            if let Some(index) = index {
                let value = Value::Number(surfaces[index]);
                if reference.unit.as_deref() == Some("percent over 100") {
                    return Ok(value);
                }
                return super::units::convert(
                    &value,
                    Some("percent over 100"),
                    reference.unit.as_deref(),
                )
                .ok_or_else(|| {
                    Error::Variable(format!(
                        "unsupported control surface unit for {}",
                        reference.name
                    ))
                });
            }
        }
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

mod events;
#[cfg(test)]
mod tests;

impl esp_physics::variables::VariableStore for Variables {
    fn set(&mut self, category: char, name: &str, unit: Option<&str>, value: Value) {
        Variables::set(self, category, name, unit, value)
    }
    fn set_number(&mut self, name: &str, unit: &str, value: f64) {
        Variables::set_number(self, name, unit, value)
    }
    fn dispatch_key_event(&mut self, id: u32, value: Value) -> Result<(), Error> {
        Variables::dispatch_key_event(self, id, value)
    }
}
