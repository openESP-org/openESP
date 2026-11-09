use super::*;

impl Flight {
    pub fn publish_loading(&self, variables: &mut Variables) {
        variables.set_number("SIM ON GROUND", "bool", f64::from(self.on_ground));
        variables.set_number("STRUCTURAL ICE PCT", "percent", self.structural_ice * 100.0);
        let surface = self.ground.as_ref().and_then(|ground| {
            ground.surface_type(self.current.position.x, self.current.position.y)
        });
        variables.set_number(
            "OPENESP GROUND SURFACE",
            "number",
            surface.map_or(-1.0, f64::from),
        );
        let contacts = self.gear.sound_contacts(
            &self.current,
            self.loading.cg,
            self.surface_at(&self.current),
        );
        for (index, contact) in contacts.into_iter().enumerate() {
            variables.set_number(
                &format!("OPENESP GEAR SOUND CONTACT:{index}"),
                "bool",
                f64::from(contact),
            );
        }
        variables.set_number(
            "AUTO SPOILERS AVAILABLE",
            "bool",
            f64::from(self.auto_spoiler_available),
        );
        variables.set_number(
            "IS GEAR RETRACTABLE",
            "bool",
            f64::from(self.gear.retractable()),
        );
        variables.set_number(
            "GEAR TOTAL PCT EXTENDED",
            "percent over 100",
            self.gear.extension(),
        );
        for (index, position) in self.gear.animation_positions(
            &self.current,
            self.loading.cg,
            self.surface_at(&self.current),
        ) {
            variables.set_number(
                &format!("GEAR ANIMATION POSITION:{index}"),
                "keyframe",
                position,
            );
        }
        for (index, position) in self.gear.extensions() {
            let state = if position == 0.0 {
                1.0
            } else if position == 1.0 {
                2.0
            } else {
                0.0
            };
            variables.set_number(&format!("GEAR POSITION:{index}"), "enum", state);
            if let Some(name) = [
                "GEAR CENTER POSITION",
                "GEAR LEFT POSITION",
                "GEAR RIGHT POSITION",
                "GEAR AUX POSITION",
            ]
            .get(index as usize)
            {
                variables.set_number(name, "percent over 100", position);
            }
        }
        for wheel in self.gear.wheel_motion() {
            variables.set_number(
                &format!("WHEEL ROTATION ANGLE:{}", wheel.index),
                "radians",
                wheel.rotation,
            );
            variables.set_number(&format!("WHEEL RPM:{}", wheel.index), "rpm", wheel.rpm);
            variables.set_number(
                &format!("GEAR STEER ANGLE:{}", wheel.index),
                "radians",
                wheel.steer_angle,
            );
            if let Some(side) = ["CENTER", "LEFT", "RIGHT", "AUX"].get(wheel.index as usize) {
                variables.set_number(
                    &format!("{side} WHEEL ROTATION ANGLE"),
                    "radians",
                    wheel.rotation,
                );
                variables.set_number(&format!("{side} WHEEL RPM"), "rpm", wheel.rpm);
                variables.set_number(
                    &format!("GEAR {side} STEER ANGLE"),
                    "radians",
                    wheel.steer_angle,
                );
            }
        }
        for (tank, &quantity) in self.weights.tanks().iter().zip(&self.loading.tank_gallons) {
            let capacity = tank.capacity_gallons + tank.unusable_gallons;
            let fraction = if capacity > 0.0 {
                quantity / capacity
            } else {
                0.0
            };
            let label = match tank.name {
                "Center1" => "CENTER",
                "Center2" => "CENTER2",
                "Center3" => "CENTER3",
                "LeftMain" => "LEFT MAIN",
                "LeftAux" => "LEFT AUX",
                "LeftTip" => "LEFT TIP",
                "RightMain" => "RIGHT MAIN",
                "RightAux" => "RIGHT AUX",
                "RightTip" => "RIGHT TIP",
                "External1" => "EXTERNAL1",
                "External2" => "EXTERNAL2",
                _ => continue,
            };
            variables.set_number(&format!("FUEL TANK {label} CAPACITY"), "gallons", capacity);
            variables.set_number(&format!("FUEL TANK {label} QUANTITY"), "gallons", quantity);
            variables.set_number(
                &format!("FUEL TANK {label} LEVEL"),
                "percent over 100",
                fraction,
            );
        }
        variables.set_number("TOTAL WEIGHT", "kilograms", self.loading.mass.kilograms());
        variables.set_number("EMPTY WEIGHT", "kilograms", self.loading.empty_kilograms);
        variables.set_number("FUEL TOTAL QUANTITY", "gallons", self.loading.fuel_gallons);
        variables.set_number(
            "FUEL TOTAL CAPACITY",
            "gallons",
            self.loading.fuel_capacity_gallons,
        );
        variables.set_number(
            "FUEL TOTAL QUANTITY WEIGHT",
            "pounds",
            self.loading.fuel_gallons * self.loading.fuel_pounds_per_gallon,
        );
        variables.set_number(
            "FUEL WEIGHT PER GALLON",
            "pounds",
            self.loading.fuel_pounds_per_gallon,
        );
    }
}
