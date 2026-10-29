use super::Error;
use binrw::BinRead;
use std::io::Cursor;

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Part {
    pub primitive_type: u32,
    pub scene_node: i32,
    pub material: i32,
    pub vertex_buffer: i32,
    pub vertex_offset: u32,
    pub vertex_count: u32,
    pub index_offset: u32,
    pub index_count: u32,
    pub mouse_rectangle: i32,
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Tangent {
    pub first: [f32; 3],
    pub second: [f32; 3],
}

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct SkinWeight {
    pub bones: [u8; 4],
    pub weights: [u8; 4],
}

#[derive(Debug, Clone, PartialEq)]
pub struct VertexBufferData<T> {
    pub buffer_index: u32,
    pub entries: Vec<T>,
}

pub fn tangents(data: &[u8]) -> Result<VertexBufferData<Tangent>, Error> {
    buffer_data(data, 24)
}

pub fn skin_weights(data: &[u8]) -> Result<VertexBufferData<SkinWeight>, Error> {
    buffer_data(data, 8)
}

pub fn bone_map(data: &[u8]) -> Result<VertexBufferData<i32>, Error> {
    buffer_data(data, 4)
}

fn buffer_data<T>(data: &[u8], size: usize) -> Result<VertexBufferData<T>, Error>
where
    T: for<'a> BinRead<Args<'a> = ()>,
{
    let mut cursor = Cursor::new(data);
    let buffer_index = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
    Ok(VertexBufferData {
        buffer_index,
        entries: records(&data[4..], size)?,
    })
}

pub fn vertices(data: &[u8]) -> Result<Vec<Vertex>, Error> {
    records(data, 32)
}

pub fn indices(data: &[u8]) -> Result<Vec<u16>, Error> {
    records(data, 2)
}

pub fn parts(data: &[u8]) -> Result<Vec<Part>, Error> {
    records(data, 36)
}

pub fn transforms(data: &[u8]) -> Result<Vec<[f32; 16]>, Error> {
    records(data, 64)
}

pub(super) fn records<T>(data: &[u8], size: usize) -> Result<Vec<T>, Error>
where
    T: for<'a> BinRead<Args<'a> = ()>,
{
    if !data.len().is_multiple_of(size) {
        return Err(Error::InvalidSize);
    }
    data.chunks_exact(size)
        .map(|record| T::read_le(&mut Cursor::new(record)).map_err(|_| Error::InvalidSize))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_skin_weights_without_normalizing_bytes() {
        let buffer = skin_weights(&[2, 0, 0, 0, 1, 2, 3, 4, 255, 0, 128, 64]).unwrap();
        assert_eq!(buffer.buffer_index, 2);
        assert_eq!(buffer.entries[0].bones, [1, 2, 3, 4]);
        assert_eq!(buffer.entries[0].weights, [255, 0, 128, 64]);
        assert!(skin_weights(&[0; 3]).is_err());
        assert_eq!(skin_weights(&[0; 11]), Err(Error::InvalidSize));
        assert_eq!(
            bone_map(&[1, 0, 0, 0, 255, 255, 255, 255]).unwrap().entries,
            [-1]
        );
    }

    #[test]
    fn reads_tangent_vectors_after_buffer_index() {
        let mut bytes = 3u32.to_le_bytes().to_vec();
        for value in [1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0] {
            bytes.extend(value.to_le_bytes());
        }
        let buffer = tangents(&bytes).unwrap();
        assert_eq!(buffer.buffer_index, 3);
        assert_eq!(
            buffer.entries,
            [Tangent {
                first: [1.0, 2.0, 3.0],
                second: [4.0, 5.0, 6.0]
            }]
        );
        assert_eq!(tangents(&bytes[..27]), Err(Error::InvalidSize));
        assert!(tangents(&[0; 4]).unwrap().entries.is_empty());
    }

    #[test]
    fn decodes_vertices_without_changing_coordinates() {
        let bytes: Vec<_> = [1.0f32, -2.0, 3.0, 0.0, 1.0, 0.0, 0.25, 0.75]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        assert_eq!(
            vertices(&bytes).unwrap(),
            [Vertex {
                position: [1.0, -2.0, 3.0],
                normal: [0.0, 1.0, 0.0],
                uv: [0.25, 0.75],
            }]
        );
        for end in 1..32 {
            assert_eq!(vertices(&bytes[..end]), Err(Error::InvalidSize));
        }
    }

    #[test]
    fn retains_indices_and_unknown_primitive_types() {
        assert_eq!(indices(&[0, 0, 255, 255]).unwrap(), [0, 65535]);
        assert_eq!(indices(&[0]), Err(Error::InvalidSize));
        let bytes: Vec<_> = [99i32, -1, 2, 3, 4, 5, 6, 7, -1]
            .into_iter()
            .flat_map(i32::to_le_bytes)
            .collect();
        let part = parts(&bytes).unwrap().remove(0);
        assert_eq!(part.primitive_type, 99);
        assert_eq!(part.scene_node, -1);
        assert_eq!(part.vertex_offset, 4);
        assert_eq!(part.index_count, 7);
        assert_eq!(part.mouse_rectangle, -1);
        assert_eq!(parts(&bytes[..35]), Err(Error::InvalidSize));
    }

    #[test]
    fn reads_matrices_and_empty_buffers() {
        let bytes: Vec<_> = (0..16).flat_map(|i| (i as f32).to_le_bytes()).collect();
        let matrices = transforms(&bytes).unwrap();
        assert_eq!(matrices[0][15], 15.0);
        assert_eq!(transforms(&bytes[..63]), Err(Error::InvalidSize));
        assert!(vertices(&[]).unwrap().is_empty());
        assert!(parts(&[]).unwrap().is_empty());
    }
}
