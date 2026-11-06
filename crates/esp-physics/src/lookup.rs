use anyhow::{Result, ensure};
use esp_formats::air::{
    Record,
    table::{Point, Table},
};

pub struct Curve(pub(crate) Vec<Point>);

impl Curve {
    pub fn from_values(values: &[(f64, f64)]) -> Result<Self> {
        Self::new(
            values
                .iter()
                .map(|&(input, output)| Point { input, output })
                .collect(),
        )
    }

    pub fn outputs_nonnegative(&self) -> bool {
        self.0.iter().all(|point| point.output >= 0.0)
    }
    pub fn new(points: Vec<Point>) -> Result<Self> {
        ensure!(
            points.len() >= 2
                && points
                    .iter()
                    .all(|p| p.input.is_finite() && p.output.is_finite())
                && points.windows(2).all(|p| p[0].input < p[1].input),
            "invalid AIR curve"
        );
        Ok(Self(points))
    }

    pub fn from_record(record: &Record<'_>) -> Result<Self> {
        let Some(Table::Curve(table)) = record.table()? else {
            anyhow::bail!("AIR record {:#x} is not a curve", record.id);
        };
        Self::new(table.points)
    }

    pub fn sample(&self, value: f64) -> f64 {
        let i = self.0.partition_point(|p| p.input < value);
        if i == 0 {
            return self.0[0].output;
        }
        if i == self.0.len() {
            return self.0[i - 1].output;
        }
        let (a, b) = (self.0[i - 1], self.0[i]);
        a.output + (b.output - a.output) * (value - a.input) / (b.input - a.input)
    }

    pub fn inverse(&self, value: f64) -> f64 {
        let i = self.0.partition_point(|p| p.output < value);
        if i == 0 {
            return self.0[0].input;
        }
        if i == self.0.len() {
            return self.0[i - 1].input;
        }
        let (a, b) = (self.0[i - 1], self.0[i]);
        a.input + (b.input - a.input) * (value - a.output) / (b.output - a.output)
    }
}

pub struct Grid {
    pub corner: f64,
    rows: Vec<f64>,
    columns: Vec<f64>,
    values: Vec<f64>,
}

impl Grid {
    pub fn from_record(record: &Record<'_>) -> Result<Self> {
        let Some(Table::Surface(table)) = record.table()? else {
            anyhow::bail!("AIR record {:#x} is not a surface", record.id);
        };
        Self::from_values(table.rows as usize, table.columns as usize, &table.values)
    }

    pub fn from_values(rows: usize, columns: usize, values: &[f64]) -> Result<Self> {
        ensure!(
            rows >= 3
                && columns >= 3
                && rows.checked_mul(columns) == Some(values.len())
                && values.iter().all(|v| v.is_finite()),
            "invalid AIR surface dimensions or values"
        );
        let row_inputs = (1..rows).map(|r| values[r * columns]).collect::<Vec<_>>();
        let column_inputs = values[1..columns].to_vec();
        ensure!(
            [&row_inputs, &column_inputs]
                .iter()
                .all(|axis| axis.windows(2).all(|p| p[0] < p[1])),
            "invalid AIR surface axes"
        );
        Ok(Self {
            corner: values[0],
            rows: row_inputs,
            columns: column_inputs,
            values: (1..rows)
                .flat_map(|r| values[r * columns + 1..(r + 1) * columns].iter().copied())
                .collect(),
        })
    }

    pub fn sample(&self, row: f64, column: f64) -> f64 {
        let interval = |axis: &[f64], value: f64| {
            let value = value.clamp(axis[0], axis[axis.len() - 1]);
            let i = axis
                .partition_point(|v| *v < value)
                .clamp(1, axis.len() - 1)
                - 1;
            (i, (value - axis[i]) / (axis[i + 1] - axis[i]))
        };
        let (r, dr) = interval(&self.rows, row);
        let (c, dc) = interval(&self.columns, column);
        let at = |r, c| self.values[r * self.columns.len() + c];
        let lo = at(r, c) + (at(r, c + 1) - at(r, c)) * dc;
        let hi = at(r + 1, c) + (at(r + 1, c + 1) - at(r + 1, c)) * dc;
        lo + (hi - lo) * dr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_nonfinite_surfaces_and_duplicate_axes() {
        let mut values = [0.0f64, 0.0, 1.0, 0.0, 1.0, 2.0, 1.0, 3.0, 4.0];
        for (index, value) in [(0, f64::NAN), (4, f64::INFINITY), (2, 0.0), (6, 0.0)] {
            let old = values[index];
            values[index] = value;
            let bytes = [
                3u32.to_le_bytes().to_vec(),
                3u32.to_le_bytes().to_vec(),
                values.into_iter().flat_map(f64::to_le_bytes).collect(),
            ]
            .concat();
            assert!(
                Grid::from_record(&Record {
                    id: 0x1503,
                    offset: 0,
                    data: &bytes
                })
                .is_err()
            );
            values[index] = old;
        }
    }

    #[test]
    fn interpolates_non_square_surface_with_unambiguous_axis_order() {
        let values = [
            0.9f64, 1.0, 4.0, 0.0, 10.0, 40.0, 10.0, 30.0, 60.0, 20.0, 50.0, 80.0,
        ];
        let bytes = [
            4u32.to_le_bytes().to_vec(),
            3u32.to_le_bytes().to_vec(),
            values.into_iter().flat_map(f64::to_le_bytes).collect(),
        ]
        .concat();
        let grid = Grid::from_record(&Record {
            id: 0x1503,
            offset: 0,
            data: &bytes,
        })
        .unwrap();
        assert_eq!(grid.corner, 0.9);
        assert_eq!(grid.sample(5.0, 2.5), 35.0);
        assert_eq!(grid.sample(-100.0, 100.0), 40.0);
        assert_eq!(grid.sample(100.0, -100.0), 50.0);
    }

    #[test]
    fn inverse_chooses_first_reachable_value_on_a_plateau() {
        let curve = Curve::new(vec![
            Point {
                input: 0.0,
                output: 0.0,
            },
            Point {
                input: 1.0,
                output: 50.0,
            },
            Point {
                input: 2.0,
                output: 50.0,
            },
            Point {
                input: 3.0,
                output: 100.0,
            },
        ])
        .unwrap();
        assert_eq!(curve.inverse(50.0), 1.0);
        assert_eq!(curve.inverse(75.0), 2.5);
        assert_eq!(curve.sample(curve.inverse(75.0)), 75.0);
    }
}
