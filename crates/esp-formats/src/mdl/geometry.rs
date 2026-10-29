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

fn records<T>(data: &[u8], size: usize) -> Result<Vec<T>, Error>
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
