//! Transitional AIR-table adapters; numerical lookup types remain format-independent.
use anyhow::Result;
use esp_formats::air::{Record, table::Table};

use crate::lookup::Curve;
#[cfg(test)]
use crate::lookup::Grid;

pub(crate) fn curve(record: &Record<'_>) -> Result<Curve> {
    let Some(Table::Curve(table)) = record.table()? else {
        anyhow::bail!("AIR record {:#x} is not a curve", record.id);
    };
    Curve::from_values(
        &table
            .points
            .iter()
            .map(|point| (point.input, point.output))
            .collect::<Vec<_>>(),
    )
}

#[cfg(test)]
pub(crate) fn grid(record: &Record<'_>) -> Result<Grid> {
    let Some(Table::Surface(table)) = record.table()? else {
        anyhow::bail!("AIR record {:#x} is not a surface", record.id);
    };
    Grid::from_values(table.rows as usize, table.columns as usize, &table.values)
}
