use super::{Bank, Error, value};
use binrw::BinRead;
use std::{collections::HashMap, io::Cursor};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Definition {
    Set,
    Property(value::Type),
    Unsupported,
}

#[derive(Debug, PartialEq)]
pub enum Content<'a> {
    Absent,
    Set,
    Property(value::Value<'a>),
}

#[derive(Debug, PartialEq)]
pub struct Node<'a> {
    pub tag: i32,
    pub offset: usize,
    pub parent: Option<usize>,
    pub content: Content<'a>,
}

#[derive(Debug)]
pub struct Document<'a> {
    pub nodes: Vec<Node<'a>>,
    pub trailing_data: &'a [u8],
}

impl<'a> Bank<'a> {
    pub fn decode(&self, definitions: &HashMap<Uuid, Definition>) -> Result<Document<'a>, Error> {
        self.decode_with_node_limit(definitions, usize::MAX)
    }

    pub fn decode_with_node_limit(
        &self,
        definitions: &HashMap<Uuid, Definition>,
        limit: usize,
    ) -> Result<Document<'a>, Error> {
        let mut nodes = Vec::new();
        let mut parents: Vec<(usize, usize)> = Vec::new();
        let mut position = 0;
        loop {
            while parents.last().is_some_and(|&(_, end)| position == end) {
                parents.pop();
            }
            if !nodes.is_empty() && parents.is_empty() {
                break;
            }
            if nodes.len() == limit {
                return Err(Error::NodeLimit(limit));
            }
            let parent = parents.last().map(|&(index, _)| index);
            let end = parents.last().map_or(self.body.len(), |&(_, end)| end);
            let offset = position;
            let tag_index = read_integer(self.body, &mut position, end)?;
            let content = match self.tag(tag_index)? {
                None => Content::Absent,
                Some(tag) => match definitions
                    .get(&tag.guid)
                    .ok_or(Error::MissingDefinition(tag.guid))?
                {
                    Definition::Set => {
                        let size = read_integer(self.body, &mut position, end)?;
                        let available = end - position;
                        let length = usize::try_from(size)
                            .ok()
                            .filter(|&length| length <= available)
                            .ok_or(Error::SetSize {
                                offset,
                                size,
                                available,
                            })?;
                        parents.push((nodes.len(), position + length));
                        Content::Set
                    }
                    Definition::Property(ty) => {
                        let (value, remaining) = value::parse(*ty, &self.body[position..end])
                            .map_err(|error| match error {
                                Error::Truncated { offset } => Error::Truncated {
                                    offset: position + offset,
                                },
                                error => error,
                            })?;
                        position = end - remaining.len();
                        Content::Property(value)
                    }
                    Definition::Unsupported => return Err(Error::UnsupportedDefinition(tag.guid)),
                },
            };
            nodes.push(Node {
                tag: tag_index,
                offset,
                parent,
                content,
            });
        }
        Ok(Document {
            nodes,
            trailing_data: &self.body[position..],
        })
    }
}

fn read_integer(data: &[u8], position: &mut usize, end: usize) -> Result<i32, Error> {
    let value = i32::read_le(&mut Cursor::new(&data[*position..end]))
        .map_err(|_| Error::Truncated { offset: *position })?;
    *position += 4;
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn definitions() -> HashMap<Uuid, Definition> {
        [
            (Uuid::from_u128(1), Definition::Set),
            (Uuid::from_u128(2), Definition::Property(value::Type::Long)),
            (
                Uuid::from_u128(3),
                Definition::Property(value::Type::Double),
            ),
        ]
        .into()
    }

    fn file(body: &[u8]) -> Vec<u8> {
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
        bytes.extend(body);
        bytes
    }

    fn set(children: &[u8]) -> Vec<u8> {
        [
            1i32.to_le_bytes().as_slice(),
            &(children.len() as i32).to_le_bytes(),
            children,
        ]
        .concat()
    }

    #[test]
    fn decodes_nested_sets_repeated_properties_and_absent_elements() {
        let property = [2i32.to_le_bytes(), (-42i32).to_le_bytes()].concat();
        let children = [property.clone(), set(&property), vec![0; 4]].concat();
        let bytes = file(&[set(&children), vec![0xaa; 3]].concat());
        let bank = Bank::parse_fsx(&bytes).unwrap();
        let document = bank.decode(&definitions()).unwrap();
        assert_eq!(
            document
                .nodes
                .iter()
                .map(|node| node.parent)
                .collect::<Vec<_>>(),
            [None, Some(0), Some(0), Some(2), Some(0)]
        );
        assert_eq!(
            document.nodes[1].content,
            Content::Property(value::Value::Long(-42))
        );
        assert_eq!(
            document.nodes[3].content,
            Content::Property(value::Value::Long(-42))
        );
        assert_eq!(document.nodes[4].content, Content::Absent);
        assert_eq!(document.nodes[2].offset, 16);
        assert_eq!(document.trailing_data, [0xaa; 3]);
        for end in 0..bytes.len() - 3 {
            assert!(
                Bank::parse_fsx(&bytes[..end])
                    .and_then(|bank| bank.decode(&definitions()))
                    .is_err(),
                "accepted prefix {end}"
            );
        }
    }

    #[test]
    fn rejects_values_and_child_sets_that_cross_parent_boundaries() {
        let body = [
            1i32.to_le_bytes().as_slice(),
            &8i32.to_le_bytes(),
            &3i32.to_le_bytes(),
            &0f64.to_le_bytes(),
        ]
        .concat();
        let bytes = file(&body);
        assert_eq!(
            Bank::parse_fsx(&bytes)
                .unwrap()
                .decode(&definitions())
                .unwrap_err(),
            Error::Truncated { offset: 12 }
        );
        let body: Vec<_> = [1i32, 8, 1, 8, 0, 0]
            .into_iter()
            .flat_map(i32::to_le_bytes)
            .collect();
        let bytes = file(&body);
        assert!(matches!(
            Bank::parse_fsx(&bytes).unwrap().decode(&definitions()),
            Err(Error::SetSize {
                offset: 8,
                available: 0,
                ..
            })
        ));
        for size in [-1i32, i32::MAX] {
            let bytes = file(&[1i32.to_le_bytes(), size.to_le_bytes()].concat());
            assert!(matches!(
                Bank::parse_fsx(&bytes).unwrap().decode(&definitions()),
                Err(Error::SetSize { .. })
            ));
        }
        let bytes = file(&set(&[0; 3]));
        assert!(
            Bank::parse_fsx(&bytes)
                .unwrap()
                .decode(&definitions())
                .is_err()
        );
    }

    #[test]
    fn checks_definitions_tag_indices_and_node_limits() {
        let bytes = file(&set(&[0; 4]));
        let bank = Bank::parse_fsx(&bytes).unwrap();
        assert!(matches!(
            bank.decode(&HashMap::new()),
            Err(Error::MissingDefinition(_))
        ));
        assert_eq!(
            bank.decode_with_node_limit(&definitions(), 1).unwrap_err(),
            Error::NodeLimit(1)
        );
        assert_eq!(
            bank.decode_with_node_limit(&definitions(), 2)
                .unwrap()
                .nodes
                .len(),
            2
        );
        let bytes = file(&set(&4i32.to_le_bytes()));
        assert!(matches!(
            Bank::parse_fsx(&bytes).unwrap().decode(&definitions()),
            Err(Error::TagIndex { index: 4, .. })
        ));
        let bytes = file(&[0; 4]);
        let bank = Bank::parse_fsx(&bytes).unwrap();
        assert_eq!(
            bank.decode(&HashMap::new()).unwrap().nodes[0].content,
            Content::Absent
        );
    }

    #[test]
    fn handles_deeply_nested_documents_without_recursion() {
        let depth = 10000;
        let mut body = Vec::with_capacity(depth * 8);
        for level in 0..depth {
            body.extend(1i32.to_le_bytes());
            body.extend(((depth - level - 1) as i32 * 8).to_le_bytes());
        }
        let bytes = file(&body);
        let document = Bank::parse_fsx(&bytes)
            .unwrap()
            .decode(&definitions())
            .unwrap();
        assert_eq!(document.nodes.len(), depth);
        assert_eq!(document.nodes[depth - 1].parent, Some(depth - 2));
    }
}
