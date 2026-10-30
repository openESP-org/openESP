use super::{Config, ValueError, indexed_name};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum WindowKind {
    Window,
    VirtualCockpit,
}

#[derive(Debug, PartialEq)]
pub struct Panel<'a> {
    pub alias: Option<&'a str>,
    pub windows: Vec<Window<'a>>,
}

#[derive(Debug, PartialEq)]
pub struct Size {
    pub width: f64,
    pub height: Option<f64>,
}

#[derive(Debug, PartialEq)]
pub struct Window<'a> {
    pub kind: WindowKind,
    pub index: u32,
    pub section: &'a str,
    pub file: Option<&'a str>,
    pub file_1024: Option<&'a str>,
    pub file_1024_night: Option<&'a str>,
    pub texture: Option<&'a str>,
    pub size_mm: Option<Size>,
    pub pixel_size: Option<Size>,
    pub gauges: Vec<Gauge<'a>>,
}

#[derive(Debug, PartialEq)]
pub struct Gauge<'a> {
    pub index: u32,
    pub reference: &'a str,
    pub x: f64,
    pub y: f64,
    pub width: Option<f64>,
    pub height: Option<f64>,
    pub parameters: Option<&'a str>,
}

impl<'a> Panel<'a> {
    pub fn from_config(config: &'a Config) -> Result<Self, ValueError> {
        let mut windows = Vec::new();
        for (name, properties) in &config.sections {
            let Some(section) = name else { continue };
            let (kind, index) = if let Some(index) = indexed_name(section, "window")? {
                (WindowKind::Window, index)
            } else if let Some(index) = indexed_name(section, "vcockpit")? {
                (WindowKind::VirtualCockpit, index)
            } else {
                continue;
            };
            let mut gauges = Vec::new();
            for (key, value) in properties {
                if let Some(index) = indexed_name(key, "gauge")? {
                    gauges.push(parse_gauge(index, section, key, value)?);
                }
            }
            gauges.sort_by_key(|gauge| gauge.index);
            if let Some(pair) = gauges
                .windows(2)
                .find(|pair| pair[0].index == pair[1].index)
            {
                return Err(ValueError::Duplicate {
                    section: section.into(),
                    key: format!("gauge{}", pair[0].index),
                });
            }
            let size_mm = config
                .value(section, "size_mm")?
                .map(|value| parse_size(section, "size_mm", value))
                .transpose()?;
            windows.push(Window {
                kind,
                index,
                section,
                file: config.value(section, "file")?,
                file_1024: config.value(section, "file_1024")?,
                file_1024_night: config.value(section, "file_1024_night")?,
                texture: config.value(section, "texture")?,
                size_mm,
                pixel_size: config
                    .value(section, "pixel_size")?
                    .map(|value| parse_size(section, "pixel_size", value))
                    .transpose()?,
                gauges,
            });
        }
        windows.sort_by_key(|window| (window.kind, window.index));
        if let Some(pair) = windows
            .windows(2)
            .find(|pair| pair[0].kind == pair[1].kind && pair[0].index == pair[1].index)
        {
            return Err(ValueError::Duplicate {
                section: pair[0].section.into(),
                key: "section".into(),
            });
        }
        Ok(Self {
            alias: config.value("fltsim", "alias")?,
            windows,
        })
    }
}

fn invalid(section: &str, key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: section.into(),
        key: key.into(),
        value: value.into(),
    }
}

fn number(section: &str, key: &str, value: &str) -> Result<f64, ValueError> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| invalid(section, key, value))
}

fn parse_size(section: &str, key: &str, value: &str) -> Result<Size, ValueError> {
    let mut fields = value.split(',');
    let width = number(section, key, fields.next().unwrap_or_default())?;
    let height = fields
        .next()
        .map(|field| number(section, key, field))
        .transpose()?;
    if fields.next().is_some()
        || width <= 0.0
        || height.is_some_and(|height| height < 0.0 || (height == 0.0 && key != "pixel_size"))
    {
        return Err(invalid(section, key, value));
    }
    Ok(Size { width, height })
}

fn parse_gauge<'a>(
    index: u32,
    section: &str,
    key: &str,
    value: &'a str,
) -> Result<Gauge<'a>, ValueError> {
    let mut fields = value.splitn(6, ',');
    let reference = fields.next().unwrap_or_default().trim();
    if reference.is_empty() {
        return Err(invalid(section, key, value));
    }
    let x = number(section, key, fields.next().unwrap_or_default())?;
    let y = number(section, key, fields.next().unwrap_or_default())?;
    let width = fields
        .next()
        .filter(|field| !field.trim().is_empty())
        .map(|field| number(section, key, field))
        .transpose()?;
    let height = fields
        .next()
        .filter(|field| !field.trim().is_empty())
        .map(|field| number(section, key, field))
        .transpose()?;
    Ok(Gauge {
        index,
        reference,
        x,
        y,
        width,
        height,
        parameters: fields.next(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_cockpit_textures_and_optional_gauge_sizes() {
        let config = Config::parse("[Window Titles]\nWindow00=Main Panel\n[Window00]\nfile=panel.bmp\nsize_mm=1024\ngauge01=Shared!Radio, -10, 20, 200, 100, mode=1,channel=2\ngauge00=Example!Altimeter, 50, 60\n[VCOCKPIT01]\ntexture=$VC_1\nsize_mm=1024,768\ngauge100=Example!Display,0,0,512,256\ncustom=retained\n").unwrap();
        let panel = Panel::from_config(&config).unwrap();
        assert_eq!(panel.windows.len(), 2);
        let window = &panel.windows[0];
        assert_eq!(window.file, Some("panel.bmp"));
        assert_eq!(
            window.size_mm,
            Some(Size {
                width: 1024.0,
                height: None
            })
        );
        assert_eq!(window.gauges[0].width, None);
        assert_eq!(window.gauges[1].parameters, Some(" mode=1,channel=2"));
        assert_eq!(window.gauges[1].x, -10.0);
        let cockpit = &panel.windows[1];
        assert_eq!(cockpit.kind, WindowKind::VirtualCockpit);
        assert_eq!(cockpit.texture, Some("$VC_1"));
        assert_eq!(cockpit.gauges[0].index, 100);
        assert_eq!(
            config.value("VCOCKPIT01", "custom").unwrap(),
            Some("retained")
        );
    }

    #[test]
    fn retains_aliases_and_empty_dimensions() {
        let config = Config::parse(
            "[fltsim]\nalias=..\\Other\\panel\n[Window00]\ngauge00=NativeGauge,0,0,,,custom\n",
        )
        .unwrap();
        let panel = Panel::from_config(&config).unwrap();
        assert_eq!(panel.alias, Some("..\\Other\\panel"));
        assert_eq!(panel.windows[0].gauges[0].height, None);
        assert_eq!(panel.windows[0].gauges[0].parameters, Some("custom"));
        let config = Config::parse("[VCockpit01]\npixel_size=512,0").unwrap();
        assert_eq!(
            Panel::from_config(&config).unwrap().windows[0].pixel_size,
            Some(Size {
                width: 512.0,
                height: Some(0.0)
            })
        );
    }

    #[test]
    fn rejects_ambiguous_indices_and_invalid_numbers() {
        for text in [
            "[Window00]\ngauge00=Gauge,0,0\ngauge0=Other,0,0",
            "[Window00]\n[WINDOW0]\n",
            "[Window4294967296]\n",
            "[Window00]\ngauge4294967296=Gauge,0,0",
            "[Window00]\ngauge00=Gauge,0",
            "[Window00]\ngauge00=Gauge,NaN,0",
            "[Window00]\ngauge00=Gauge,0,0,1,inf",
            "[Window00]\nsize_mm=0,100",
            "[Window00]\nsize_mm=10,20,30",
        ] {
            let config = Config::parse(text).unwrap();
            assert!(Panel::from_config(&config).is_err(), "{text}");
        }
    }
}
