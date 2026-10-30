use roxmltree::{Document, Node, NodeId, ParsingOptions};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Schema {
    Legacy,
    Ace,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid gauge XML: {0}")]
    Xml(#[from] roxmltree::Error),
    #[error("unsupported gauge document: {0}")]
    Document(String),
    #[error("expected one gauge definition, found {0}")]
    GaugeCount(usize),
    #[error("invalid gauge size: {0}")]
    Size(String),
    #[error("invalid gauge image: {0}")]
    Image(String),
}

#[derive(Debug, PartialEq, Eq)]
pub struct ImageReference<'a> {
    pub name: &'a str,
    pub mask: bool,
    pub alpha: Option<bool>,
    pub transparent: Option<bool>,
    pub bright: Option<bool>,
    pub luminous: Option<bool>,
    pub bilinear: Option<bool>,
}

#[derive(Debug)]
pub struct Gauge<'input> {
    document: Document<'input>,
    root: NodeId,
    schema: Schema,
}

impl<'input> Gauge<'input> {
    pub fn parse(text: &'input str) -> Result<Self, Error> {
        Self::parse_with_node_limit(text, u32::MAX)
    }

    pub fn parse_with_node_limit(text: &'input str, nodes_limit: u32) -> Result<Self, Error> {
        let document = Document::parse_with_options(
            text,
            ParsingOptions {
                nodes_limit,
                ..ParsingOptions::default()
            },
        )?;
        let element = document.root_element();
        let (schema, root) = match element.tag_name().name() {
            "Gauge" => (Schema::Legacy, element.id()),
            "SimBase.Document" if element.attribute("Type") == Some("AceXML") => {
                let mut gauges = element
                    .children()
                    .filter(|node| node.has_tag_name("SimGauge.Gauge"));
                let gauge = gauges.next().ok_or(Error::GaugeCount(0))?;
                let count = 1 + gauges.count();
                if count != 1 {
                    return Err(Error::GaugeCount(count));
                }
                (Schema::Ace, gauge.id())
            }
            _ => return Err(Error::Document(element.tag_name().name().into())),
        };
        Ok(Self {
            document,
            root,
            schema,
        })
    }

    pub fn document(&self) -> &Document<'input> {
        &self.document
    }

    pub fn root(&self) -> Node<'_, 'input> {
        // The ID belongs to this immutable document and is set during parsing.
        self.document.get_node(self.root).unwrap()
    }

    pub fn schema(&self) -> Schema {
        self.schema
    }

    pub fn name(&self) -> Option<&str> {
        self.root().attribute(match self.schema {
            Schema::Legacy => "Name",
            Schema::Ace => "id",
        })
    }

    pub fn version(&self) -> Option<&str> {
        match self.schema {
            Schema::Legacy => self.root().attribute("Version"),
            Schema::Ace => self.document.root_element().attribute("version"),
        }
    }

    pub fn size(&self) -> Result<Option<[u32; 2]>, Error> {
        let mut sizes = self
            .root()
            .children()
            .filter(|node| node.has_tag_name("Size"));
        let Some(size) = sizes.next() else {
            return Ok(None);
        };
        if sizes.next().is_some() {
            return Err(Error::Size("duplicate Size elements".into()));
        }
        let parse = |value: &str| value.trim().parse().map_err(|_| Error::Size(value.into()));
        let dimensions = match self.schema {
            Schema::Legacy => {
                let width = size
                    .attribute("X")
                    .ok_or_else(|| Error::Size("missing X".into()))?;
                let height = size
                    .attribute("Y")
                    .ok_or_else(|| Error::Size("missing Y".into()))?;
                [parse(width)?, parse(height)?]
            }
            Schema::Ace => {
                let value = text_content(size);
                let (width, height) = value
                    .split_once(',')
                    .ok_or_else(|| Error::Size(value.clone()))?;
                [parse(width)?, parse(height)?]
            }
        };
        Ok(Some(dimensions))
    }

    pub fn images(&self) -> impl Iterator<Item = Node<'_, 'input>> {
        self.root()
            .descendants()
            .filter(|node| node.has_tag_name("Image") || node.has_tag_name("MaskImage"))
    }

    pub fn image_references(&self) -> impl Iterator<Item = Result<ImageReference<'_>, Error>> {
        self.images().map(|node| {
            let flag = |name: &str| -> Result<Option<bool>, Error> {
                let parse = |value: &str| -> Result<bool, Error> {
                    match value.trim() {
                        value
                            if value.eq_ignore_ascii_case("true")
                                || value.eq_ignore_ascii_case("yes") =>
                        {
                            Ok(true)
                        }
                        value
                            if value.eq_ignore_ascii_case("false")
                                || value.eq_ignore_ascii_case("no") =>
                        {
                            Ok(false)
                        }
                        _ => Err(Error::Image(format!("{name}={value}"))),
                    }
                };
                match self.schema {
                    Schema::Legacy => node.attribute(name).map(parse).transpose(),
                    Schema::Ace => {
                        let mut properties =
                            node.children().filter(|child| child.has_tag_name(name));
                        let value = properties.next();
                        if properties.next().is_some() {
                            return Err(Error::Image(format!("duplicate {name}")));
                        }
                        value.map(|child| parse(&text_content(child))).transpose()
                    }
                }
            };
            Ok(ImageReference {
                name: node
                    .attribute("Name")
                    .filter(|name| !name.is_empty())
                    .ok_or_else(|| Error::Image("missing Name".into()))?,
                mask: node.has_tag_name("MaskImage"),
                alpha: flag("Alpha")?,
                transparent: flag("Transparent")?,
                bright: flag("Bright")?,
                luminous: flag("Luminous")?,
                bilinear: flag("Bilinear")?,
            })
        })
    }

    pub fn art_directory(&self) -> Option<&str> {
        self.root().attribute("ArtDirectory")
    }

    pub fn macros(&self) -> impl Iterator<Item = Node<'_, 'input>> {
        self.root()
            .descendants()
            .filter(|node| node.has_tag_name("Macro"))
    }
}

pub fn text_content(node: Node<'_, '_>) -> String {
    node.children()
        .filter(Node::is_text)
        .filter_map(|node| node.text())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_legacy_gauges_without_losing_scripts_and_extensions() {
        let source = r#"<Gauge Name="Test" Version="1.0">
<Size X="320" Y="240"/>
<Macro Name="engine">1</Macro>
<Image Name="background.bmp" ImageSizes="160,120,320,240"/>
<Element><MaskImage Name="1024\mask.bmp"/><Image Name="needle.bmp"/>
<Rotate><Value> (A:VALUE, number) &gt;<!-- keep following script --> 2 &amp;&amp;<![CDATA[ if{ 3 } ]]></Value></Rotate>
</Element><Future Attribute="kept"><Data>opaque</Data></Future>
</Gauge>"#;
        let gauge = Gauge::parse(source).unwrap();
        assert_eq!(gauge.schema(), Schema::Legacy);
        assert_eq!(gauge.name(), Some("Test"));
        assert_eq!(gauge.version(), Some("1.0"));
        assert_eq!(gauge.size().unwrap(), Some([320, 240]));
        assert_eq!(gauge.images().count(), 3);
        assert_eq!(
            gauge.macros().next().unwrap().attribute("Name"),
            Some("engine")
        );
        let script = gauge
            .root()
            .descendants()
            .find(|node| node.has_tag_name("Value"))
            .unwrap();
        assert_eq!(text_content(script), " (A:VALUE, number) > 2 && if{ 3 } ");
        assert!(
            gauge
                .root()
                .descendants()
                .any(|node| node.has_tag_name("Future"))
        );
        assert_eq!(gauge.document().input_text(), source);
    }

    #[test]
    fn reads_ace_gauge_objects() {
        let gauge = Gauge::parse(
            r#"<?xml version="1.0"?>
<SimBase.Document Type="AceXML" version="1,0" id="Document">
<Filename>gauge.xml</Filename><SimGauge.Gauge id="Instrument" ArtDirectory=".">
<Size>512, 256</Size><Macro Name="offset"><MacroValue> 2 </MacroValue></Macro>
<Element id="Needle"><FloatPosition>10.5,20.0</FloatPosition>
<Image Name="needle.bmp"><Bright>True</Bright></Image>
<Rotation><Expression><Script> (A:VALUE, number) </Script></Expression></Rotation>
</Element><MouseArea><MouseClick><ClickType>LeftSingle+LeftRelease</ClickType>
<Script> 1 (&gt;K:EVENT) </Script></MouseClick></MouseArea>
</SimGauge.Gauge></SimBase.Document>"#,
        )
        .unwrap();
        assert_eq!(gauge.schema(), Schema::Ace);
        assert_eq!(gauge.name(), Some("Instrument"));
        assert_eq!(gauge.version(), Some("1,0"));
        assert_eq!(gauge.size().unwrap(), Some([512, 256]));
        assert_eq!(
            gauge.images().next().unwrap().attribute("Name"),
            Some("needle.bmp")
        );
        assert_eq!(gauge.macros().count(), 1);
    }

    #[test]
    fn rejects_malformed_documents_and_limits_nodes() {
        for source in [
            "",
            "<Gauge>",
            "<Gauge><Element></Gauge>",
            "<Gauge Name='one' Name='two'/>",
            "<Gauge>&missing;</Gauge>",
            "<!DOCTYPE Gauge [<!ENTITY value 'text'>]><Gauge>&value;</Gauge>",
            "<Gauge/><Gauge/>",
            "<Other/>",
            "<SimBase.Document Type='MissionFile'><SimGauge.Gauge/></SimBase.Document>",
            "<SimBase.Document Type='AceXML'/>",
            "<SimBase.Document Type='AceXML'><SimGauge.Gauge/><SimGauge.Gauge/></SimBase.Document>",
        ] {
            assert!(Gauge::parse(source).is_err(), "{source}");
        }
        assert!(Gauge::parse_with_node_limit("<Gauge><Element/></Gauge>", 2).is_err());
        assert!(Gauge::parse("<Gauge/>").unwrap().size().unwrap().is_none());
        for size in [
            "<Size/>",
            "<Size X='NaN' Y='1'/>",
            "<Size X='-1' Y='2'/>",
            "<Size X='1' Y='2'/><Size X='1' Y='2'/>",
        ] {
            assert!(
                Gauge::parse(&format!("<Gauge>{size}</Gauge>"))
                    .unwrap()
                    .size()
                    .is_err()
            );
        }
        for size in ["1", "1,2,3", "4294967296,2", "1.5,2"] {
            assert!(Gauge::parse(&format!("<SimBase.Document Type='AceXML'><SimGauge.Gauge><Size>{size}</Size></SimGauge.Gauge></SimBase.Document>")).unwrap().size().is_err());
        }
    }

    #[test]
    fn reads_image_modes_from_both_xml_schemas() {
        for source in [
            "<Gauge ArtDirectory='Art'><Image Name='background.bmp'/><Element><MaskImage Name='mask.bmp' Alpha='Yes' Transparent='no' Bright='TRUE' Luminous='false' Bilinear='yes'/></Element></Gauge>",
            "<SimBase.Document Type='AceXML'><SimGauge.Gauge ArtDirectory='Art'><Image Name='background.bmp'/><Element><MaskImage Name='mask.bmp'><Alpha>Tr<!--comment-->ue</Alpha><Transparent>False</Transparent><Bright>Yes</Bright><Luminous>No</Luminous><Bilinear>True</Bilinear></MaskImage></Element></SimGauge.Gauge></SimBase.Document>",
        ] {
            let gauge = Gauge::parse(source).unwrap();
            assert_eq!(gauge.art_directory(), Some("Art"));
            assert_eq!(
                gauge
                    .image_references()
                    .collect::<Result<Vec<_>, _>>()
                    .unwrap(),
                [
                    ImageReference {
                        name: "background.bmp",
                        mask: false,
                        alpha: None,
                        transparent: None,
                        bright: None,
                        luminous: None,
                        bilinear: None
                    },
                    ImageReference {
                        name: "mask.bmp",
                        mask: true,
                        alpha: Some(true),
                        transparent: Some(false),
                        bright: Some(true),
                        luminous: Some(false),
                        bilinear: Some(true)
                    },
                ]
            );
        }
    }

    #[test]
    fn reports_ambiguous_and_invalid_image_properties() {
        for image in [
            "<Image/>",
            "<Image Name=''/>",
            "<Image Name='a.bmp' Alpha='perhaps'/>",
            "<Image Name='a.bmp' Transparent=''/>",
        ] {
            let source = format!("<Gauge>{image}</Gauge>");
            let gauge = Gauge::parse(&source).unwrap();
            assert!(matches!(
                gauge.image_references().next().unwrap(),
                Err(Error::Image(_))
            ));
        }
        for property in [
            "<Alpha>True</Alpha><Alpha>False</Alpha>",
            "<Bright>1</Bright>",
        ] {
            let source = format!(
                "<SimBase.Document Type='AceXML'><SimGauge.Gauge><Image Name='a.bmp'>{property}</Image></SimGauge.Gauge></SimBase.Document>"
            );
            let gauge = Gauge::parse(&source).unwrap();
            assert!(matches!(
                gauge.image_references().next().unwrap(),
                Err(Error::Image(_))
            ));
        }
    }
}
