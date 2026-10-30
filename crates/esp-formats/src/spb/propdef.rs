use super::document::Definition;
use roxmltree::{Document, Node, NodeId, ParsingOptions};
use std::collections::HashMap;
use uuid::Uuid;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid property-definition XML: {0}")]
    Xml(#[from] roxmltree::Error),
    #[error("property-definition document contains no SymbolDef")]
    MissingSymbols,
    #[error("missing {attribute} on {element}")]
    MissingAttribute {
        element: String,
        attribute: &'static str,
    },
    #[error("ambiguous {attribute} on {element}")]
    AmbiguousAttribute {
        element: String,
        attribute: &'static str,
    },
    #[error("invalid property-definition GUID {0}")]
    Guid(String),
    #[error("duplicate property-definition GUID {0}")]
    DuplicateGuid(Uuid),
}

#[derive(Debug)]
pub struct Definitions<'input> {
    document: Document<'input>,
    by_id: HashMap<Uuid, (NodeId, Definition)>,
}

impl<'input> Definitions<'input> {
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
        let mut by_id = HashMap::new();
        let mut found = false;
        for symbol in document
            .descendants()
            .filter(|node| node.has_tag_name("SymbolDef"))
        {
            found = true;
            required(symbol, "name")?;
            for group in symbol.children().filter(Node::is_element) {
                let element = match group.tag_name().name() {
                    "SymbolInclude" => {
                        required(group, "filename")?;
                        continue;
                    }
                    "TypeDefs" => "TypeDef",
                    "PropertyDefs" => "PropertyDef",
                    "SetDefs" => "SetDef",
                    _ => continue,
                };
                for node in group.children().filter(|node| node.has_tag_name(element)) {
                    required(node, "name")?;
                    let binding = match element {
                        "SetDef" => Definition::Set,
                        "PropertyDef" => required(node, "type")?
                            .parse()
                            .map_or(Definition::Unsupported, Definition::Property),
                        _ => Definition::Unsupported,
                    };
                    let id = if element == "TypeDef" {
                        attribute(node, "id")?
                    } else {
                        Some(required(node, "id")?)
                    };
                    let Some(id) = id else { continue };
                    let guid = Uuid::parse_str(id).map_err(|_| Error::Guid(id.into()))?;
                    if by_id.insert(guid, (node.id(), binding)).is_some() {
                        return Err(Error::DuplicateGuid(guid));
                    }
                }
            }
        }
        if !found {
            return Err(Error::MissingSymbols);
        }
        Ok(Self { document, by_id })
    }

    pub fn document(&self) -> &Document<'input> {
        &self.document
    }

    pub fn lookup(&self, guid: Uuid) -> Option<Node<'_, 'input>> {
        self.by_id
            .get(&guid)
            .and_then(|&(id, _)| self.document.get_node(id))
    }

    pub fn includes(&self) -> impl Iterator<Item = &str> {
        self.document
            .descendants()
            .filter(|node| node.has_tag_name("SymbolInclude"))
            .filter_map(|node| node.attribute("filename"))
    }

    pub fn bindings(&self) -> HashMap<Uuid, Definition> {
        self.by_id
            .iter()
            .map(|(&guid, &(_, binding))| (guid, binding))
            .collect()
    }
}

fn attribute<'a>(node: Node<'a, '_>, key: &'static str) -> Result<Option<&'a str>, Error> {
    let mut matches = node.attributes().filter(|attr| match key {
        "name" => matches!(attr.name(), "name" | "Name"),
        "id" => matches!(attr.name(), "id" | "Id" | "ID"),
        "type" => matches!(attr.name(), "type" | "Type"),
        _ => attr.name() == key,
    });
    let value = matches.next().map(|attr| attr.value());
    if matches.next().is_some() {
        return Err(Error::AmbiguousAttribute {
            element: node.tag_name().name().into(),
            attribute: key,
        });
    }
    Ok(value)
}

fn required<'a>(node: Node<'a, '_>, key: &'static str) -> Result<&'a str, Error> {
    attribute(node, key)?
        .filter(|value| !value.is_empty())
        .ok_or_else(|| Error::MissingAttribute {
            element: node.tag_name().name().into(),
            attribute: key,
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spb::{Bank, Error as BinaryError, document::Content, value};

    const SCHEMA: &str = "<SimBase.Document><SymbolDef name='Test'><SymbolInclude filename='Common\\base.xml'/><TypeDefs><TypeDef name='LONG'/></TypeDefs><SetDefs><SetDef name='Root' id='{00000000-0000-0000-0000-000000000001}'><property name='Value'/></SetDef></SetDefs><PropertyDefs><PropertyDef Name='Value' ID='{00000000-0000-0000-0000-000000000002}' Type='Long' xml_io='attribute'><EnumDef><EnumVal name='zero'/><EnumVal name='one' xml_name='First'/></EnumDef></PropertyDef><PropertyDef name='Future' id='{00000000-0000-0000-0000-000000000003}' type='CUSTOM'/></PropertyDefs><Extension arbitrary='preserved'/></SymbolDef></SimBase.Document>";

    #[test]
    fn binds_properties_and_sets_without_discarding_schema_metadata() {
        let definitions = Definitions::parse(SCHEMA).unwrap();
        let bindings = definitions.bindings();
        assert_eq!(bindings[&Uuid::from_u128(1)], Definition::Set);
        assert_eq!(
            bindings[&Uuid::from_u128(2)],
            Definition::Property(value::Type::Long)
        );
        assert_eq!(bindings[&Uuid::from_u128(3)], Definition::Unsupported);
        assert_eq!(
            definitions.includes().collect::<Vec<_>>(),
            [r"Common\base.xml"]
        );
        let node = definitions.lookup(Uuid::from_u128(2)).unwrap();
        assert_eq!(node.attribute("xml_io"), Some("attribute"));
        assert_eq!(
            node.descendants()
                .filter(|node| node.has_tag_name("EnumVal"))
                .count(),
            2
        );
        assert_eq!(definitions.document().input_text(), SCHEMA);
        assert!(definitions.lookup(Uuid::nil()).is_none());
    }

    #[test]
    fn decodes_a_bank_using_parsed_property_definitions() {
        let definitions = Definitions::parse(SCHEMA).unwrap();
        let mut bytes = 0xebacu16.to_le_bytes().to_vec();
        bytes.extend(
            [0i32, 0, 0, 0, 4, 0, 0, 0, 0, 0]
                .into_iter()
                .flat_map(i32::to_le_bytes),
        );
        for identity in 1..=3 {
            bytes.extend(Uuid::from_u128(identity).to_bytes_le());
            bytes.extend(0i32.to_le_bytes());
        }
        bytes.extend([1i32, 8, 2, -42].into_iter().flat_map(i32::to_le_bytes));
        let bank = Bank::parse_fsx(&bytes).unwrap();
        let document = bank.decode(&definitions.bindings()).unwrap();
        assert_eq!(
            document.nodes[1].content,
            Content::Property(value::Value::Long(-42))
        );
        bytes[110..114].copy_from_slice(&3i32.to_le_bytes());
        assert!(matches!(
            Bank::parse_fsx(&bytes)
                .unwrap()
                .decode(&definitions.bindings()),
            Err(BinaryError::UnsupportedDefinition(_))
        ));
    }

    #[test]
    fn rejects_malformed_ambiguous_and_duplicate_definitions() {
        for source in [
            "",
            "<Other/>",
            "<SymbolDef/>",
            "<SymbolDef name='Test'><SymbolInclude/></SymbolDef>",
            "<SymbolDef name='Test'><PropertyDefs><PropertyDef name='x' id='invalid' type='LONG'/></PropertyDefs></SymbolDef>",
            "<SymbolDef name='Test'><SetDefs><SetDef name='x'/></SetDefs></SymbolDef>",
            "<SymbolDef name='Test'><SetDefs><SetDef name='x' Name='y' id='{00000000-0000-0000-0000-000000000001}'/></SetDefs></SymbolDef>",
        ] {
            assert!(Definitions::parse(source).is_err(), "{source}");
        }
        let duplicate = SCHEMA.replace("000000000003", "000000000002");
        assert!(matches!(
            Definitions::parse(&duplicate),
            Err(Error::DuplicateGuid(_))
        ));
        assert!(Definitions::parse_with_node_limit(SCHEMA, 2).is_err());
    }
}
