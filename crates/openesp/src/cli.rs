use crate::vor;
use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
pub(crate) struct Arguments {
    #[arg(long)]
    pub(crate) aircraft: PathBuf,
    #[arg(long)]
    pub(crate) fsx_root: Option<PathBuf>,
    #[arg(
        long,
        group = "scenery_source",
        help = "Load airport runways from an FSX or Prepar3D BGL file"
    )]
    pub(crate) scenery_bgl: Option<PathBuf>,
    #[arg(long, group = "scenery_source", conflicts_with_all = ["scenery_bgl", "scenery_objects", "scenery_library", "scenery_textures"], help = "Discover airport, placements and libraries in an add-on folder; centre the local world at the selected airport")]
    pub(crate) scenery: Option<PathBuf>,
    #[arg(
        long,
        requires = "scenery_source",
        help = "Select an airport by ICAO code from the BGL"
    )]
    pub(crate) airport: Option<String>,
    #[arg(long, requires_all = ["airport", "airspeed"], conflicts_with_all = ["parking", "heading", "start_north", "start_east", "approach_distance"], help = "Spawn at a runway end, for example 06 or 24L")]
    pub(crate) runway: Option<String>,
    #[arg(long, requires_all = ["airport", "airspeed"], conflicts_with_all = ["runway", "heading", "start_north", "start_east", "approach_distance"], help = "Spawn at a zero-based parking stand index from the airport BGL")]
    pub(crate) parking: Option<usize>,
    #[arg(long, help = "Load placed library objects from a scenery BGL")]
    pub(crate) scenery_objects: Option<PathBuf>,
    #[arg(long, help = "Additional BGL model library; may be repeated")]
    pub(crate) scenery_library: Vec<PathBuf>,
    #[arg(long, help = "Texture directory for scenery model libraries")]
    pub(crate) scenery_textures: Option<PathBuf>,
    #[arg(long, default_value_t = 0)]
    pub(crate) variant: u32,
    #[arg(long)]
    pub(crate) cockpit: bool,
    #[arg(long, default_value_t = 4, value_parser = parse_msaa, help = "MSAA samples: 1 (off), 2, 4, or 8")]
    pub(crate) msaa: u32,
    #[arg(long, value_parser = parse_utc_time, help = "Set starting UTC time as HH:MM or HH:MM:SS")]
    pub(crate) utc_time: Option<f64>,
    #[arg(
        long,
        alias = "glide",
        help = "Enable flight physics at this airspeed in knots in the local test scene"
    )]
    pub(crate) airspeed: Option<f64>,
    #[arg(
        long,
        requires = "airspeed",
        help = "Start with the configured engines running at idle"
    )]
    pub(crate) engines_running: bool,
    #[arg(
        long,
        default_value_t = 1000.0,
        help = "Starting height in meters above the test grid; zero starts on the landing gear"
    )]
    pub(crate) height: f64,
    #[arg(
        long,
        requires = "airspeed",
        allow_negative_numbers = true,
        help = "Starting true heading in degrees"
    )]
    pub(crate) heading: Option<f64>,
    #[arg(
        long,
        requires = "airspeed",
        allow_negative_numbers = true,
        help = "Starting north offset from the test grid origin in meters"
    )]
    pub(crate) start_north: Option<f64>,
    #[arg(
        long,
        requires = "airspeed",
        allow_negative_numbers = true,
        help = "Starting east offset from the test grid origin in meters"
    )]
    pub(crate) start_east: Option<f64>,
    #[arg(long, requires_all = ["airspeed", "ils_course"], conflicts_with_all = ["heading", "start_north", "start_east", "height"], help = "Start on the ILS glideslope this many nautical miles before the threshold (0.5 to 10)")]
    pub(crate) approach_distance: Option<f64>,
    #[arg(
        long,
        default_value_t = 0.0,
        help = "Wind direction in degrees true, measured from where the wind comes"
    )]
    pub(crate) wind_direction: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        allow_negative_numbers = true,
        help = "Latitude of the local test grid origin in degrees"
    )]
    pub(crate) latitude: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        allow_negative_numbers = true,
        help = "Longitude of the local test grid origin in degrees"
    )]
    pub(crate) longitude: f64,
    #[arg(
        long,
        requires = "destination_longitude",
        allow_negative_numbers = true,
        help = "Direct-to destination latitude in degrees"
    )]
    pub(crate) destination_latitude: Option<f64>,
    #[arg(
        long,
        requires = "destination_latitude",
        allow_negative_numbers = true,
        help = "Direct-to destination longitude in degrees"
    )]
    pub(crate) destination_longitude: Option<f64>,
    #[arg(
        long,
        requires = "vor_longitude",
        allow_negative_numbers = true,
        help = "Local VOR station latitude in degrees"
    )]
    pub(crate) vor_latitude: Option<f64>,
    #[arg(
        long,
        requires = "vor_latitude",
        allow_negative_numbers = true,
        help = "Local VOR station longitude in degrees"
    )]
    pub(crate) vor_longitude: Option<f64>,
    #[arg(long, default_value_t = 113.0, help = "Local VOR frequency in MHz")]
    pub(crate) vor_frequency: f64,
    #[arg(
        long,
        default_value_t = 40.0,
        help = "Local VOR reception range in nautical miles"
    )]
    pub(crate) vor_range: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        allow_negative_numbers = true,
        help = "Primary VOR or ILS station elevation in meters above the test grid"
    )]
    pub(crate) vor_elevation: f64,
    #[arg(
        long = "vor-station",
        value_name = "LAT,LON,MHZ,RANGE_NM[,ELEVATION_M]",
        allow_hyphen_values = true,
        help = "Add a VOR station; may be repeated"
    )]
    pub(crate) vor_stations: Vec<vor::Station>,
    #[arg(
        long,
        requires = "vor_latitude",
        allow_negative_numbers = true,
        help = "Use the local station as an ILS with this inbound magnetic course"
    )]
    pub(crate) ils_course: Option<f64>,
    #[arg(long, default_value_t = 3.0, help = "ILS glideslope angle in degrees")]
    pub(crate) ils_glideslope: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        allow_negative_numbers = true,
        help = "Magnetic declination in degrees, positive east and negative west"
    )]
    pub(crate) magnetic_declination: f64,
    #[arg(long, default_value_t = 0.0, help = "Wind speed in knots")]
    pub(crate) wind_speed: f64,
    #[arg(
        long,
        default_value_t = 1013.25,
        help = "Sea-level pressure in hectopascals"
    )]
    pub(crate) sea_level_pressure: f64,
    #[arg(
        long,
        default_value_t = 15.0,
        allow_negative_numbers = true,
        help = "Sea-level temperature in degrees Celsius"
    )]
    pub(crate) sea_level_temperature: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        requires = "airspeed",
        help = "Fill usable fuel tanks to this percentage for the airborne test"
    )]
    pub(crate) fuel_percent: f64,
    #[arg(
        long,
        requires = "airspeed",
        help = "Load the aircraft's configured payload stations for the airborne test"
    )]
    pub(crate) payload: bool,
    #[arg(long, value_parser = clap::value_parser!(u64).range(1..))]
    pub(crate) frames: Option<u64>,
    #[arg(
        long,
        help = "Save the final requested frame, or the first frame without --frames"
    )]
    pub(crate) screenshot: Option<PathBuf>,
    #[arg(long = "variable", value_parser = parse_variable)]
    pub(crate) variables: Vec<Variable>,
    #[arg(long = "event", value_parser = parse_event, help = "Send a startup simulator event: NAME or NAME=VALUE")]
    pub(crate) events: Vec<StartupEvent>,
}

#[derive(Clone)]
pub(crate) struct StartupEvent {
    pub(crate) name: String,
    pub(crate) value: f64,
}

fn parse_msaa(text: &str) -> Result<u32, String> {
    match text.parse::<u32>() {
        Ok(value @ (1 | 2 | 4 | 8)) => Ok(value),
        _ => Err("MSAA must be 1, 2, 4, or 8".into()),
    }
}
pub(crate) fn parse_utc_time(text: &str) -> Result<f64, String> {
    let parts = text.split(':').collect::<Vec<_>>();
    if !(2..=3).contains(&parts.len())
        || parts
            .iter()
            .any(|part| part.len() != 2 || !part.bytes().all(|b| b.is_ascii_digit()))
    {
        return Err("expected UTC time HH:MM or HH:MM:SS".into());
    }
    let hour = parts[0].parse::<u32>().map_err(|e| e.to_string())?;
    let minute = parts[1].parse::<u32>().map_err(|e| e.to_string())?;
    let second = if parts.len() == 3 {
        parts[2].parse::<u32>().map_err(|e| e.to_string())?
    } else {
        0
    };
    if hour >= 24 || minute >= 60 || second >= 60 {
        return Err("invalid UTC time".into());
    }
    Ok(f64::from(hour * 3600 + minute * 60 + second))
}

pub(crate) fn parse_event(text: &str) -> Result<StartupEvent, String> {
    let (name, value) = text.split_once('=').unwrap_or((text, "0"));
    let name = name.trim();
    if name.is_empty() {
        return Err("event name is empty".into());
    }
    let value = value.trim().parse::<f64>().map_err(|e| e.to_string())?;
    if !value.is_finite() {
        return Err("event value must be finite".into());
    }
    Ok(StartupEvent {
        name: name.to_ascii_uppercase(),
        value,
    })
}

#[derive(Clone)]
pub(crate) struct Variable {
    pub(crate) category: char,
    pub(crate) name: String,
    pub(crate) unit: String,
    pub(crate) value: f64,
}

fn parse_variable(text: &str) -> Result<Variable, String> {
    let (reference, value) = text
        .rsplit_once('=')
        .ok_or("expected CATEGORY:NAME,UNIT=VALUE")?;
    let (category, name) = reference
        .split_once(':')
        .ok_or("expected variable category")?;
    let (name, unit) = name.rsplit_once(',').ok_or("expected variable unit")?;
    let mut chars = category.chars();
    let category = chars.next().ok_or("empty variable category")?;
    if chars.next().is_some() {
        return Err("invalid variable category".into());
    }
    let value = value.parse::<f64>().map_err(|e| e.to_string())?;
    if !value.is_finite() {
        return Err("variable must be finite".into());
    }
    Ok(Variable {
        category,
        name: name.into(),
        unit: unit.into(),
        value,
    })
}
