use anyhow::{Context, Result, bail};
use esp_formats::{cfg, mdl};
use glam::{Mat4, Vec3};
use std::{
    collections::HashSet,
    path::{Component, Path, PathBuf},
};

pub mod animation;
pub mod texture;

pub struct Aircraft {
    pub title: String,
    pub exterior: Model,
    pub interior: Option<Model>,
    pub eyepoint: Option<Vec3>,
    pub texture_directories: Vec<PathBuf>,
}

pub struct Model {
    pub nodes: Vec<Node>,
    pub materials: Vec<Material>,
    pub lods: Vec<Lod>,
    pub animations: Vec<animation::Animation>,
    pub visibility: Vec<animation::Visibility>,
}

pub struct Node {
    pub parent: Option<usize>,
    pub transform: Mat4,
    pub animation: Option<usize>,
    pub visibility: Option<usize>,
}

pub struct Material {
    pub source: mdl::material::Material,
    pub diffuse_texture: Option<String>,
}

pub struct Lod {
    pub level: i32,
    pub meshes: Vec<Mesh>,
}

pub struct Mesh {
    pub vertices: Vec<mdl::geometry::Vertex>,
    pub indices: Vec<u16>,
    pub node: usize,
    pub material: usize,
    pub mouse_rectangle: i32,
    pub skin_weights: Option<Vec<mdl::geometry::SkinWeight>>,
    pub bones: Option<Vec<usize>>,
}

pub fn resolve_path(base: &Path, name: &str) -> Result<PathBuf> {
    let normalized = name.replace('\\', "/");
    let mut path = base.to_path_buf();
    for component in Path::new(&normalized).components() {
        match component {
            Component::Normal(name) => {
                let mut found = None;
                for entry in std::fs::read_dir(&path)
                    .with_context(|| format!("reading {}", path.display()))?
                {
                    let entry = entry?;
                    if entry
                        .file_name()
                        .to_string_lossy()
                        .eq_ignore_ascii_case(&name.to_string_lossy())
                    {
                        if found.is_some() {
                            bail!("ambiguous path {} in {}", name.display(), path.display());
                        }
                        found = Some(entry.path());
                    }
                }
                path = found
                    .with_context(|| format!("missing {} in {}", name.display(), path.display()))?;
            }
            Component::ParentDir => {
                path.pop();
            }
            Component::CurDir => {}
            Component::RootDir | Component::Prefix(_) => path.push(component.as_os_str()),
        }
    }
    Ok(path)
}

fn config(path: &Path) -> Result<cfg::Config> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    cfg::Config::from_bytes(&bytes, encoding_rs::WINDOWS_1252)
        .with_context(|| format!("parsing {}", path.display()))
}

fn variant_directory(root: &Path, kind: &str, suffix: Option<&str>) -> Result<PathBuf> {
    let name = match suffix.filter(|s| !s.is_empty()) {
        Some(suffix) => format!("{kind}.{suffix}"),
        None => kind.into(),
    };
    resolve_path(root, &name)
}

impl Aircraft {
    pub fn load(root: &Path, variant: u32) -> Result<Self> {
        let aircraft = config(&resolve_path(root, "aircraft.cfg")?)?;
        let variants = cfg::aircraft::configurations(&aircraft)?;
        let variant = variants
            .iter()
            .find(|v| v.index == variant)
            .with_context(|| format!("aircraft variant {variant} is absent"))?;
        let directory = variant_directory(root, "model", variant.model)?;
        let models = config(&resolve_path(&directory, "model.cfg")?)?;
        let models = cfg::model::Models::from_config(&models)?;
        if models.alias.is_some() {
            bail!("model aliases are not supported yet");
        }
        let load = |name: &str| Model::load(&resolve_path(&directory, &format!("{name}.mdl"))?);
        let exterior = load(models.normal.context("model.cfg has no exterior model")?)?;
        let interior = models.interior.map(load).transpose()?;
        let texture_directory = variant_directory(root, "texture", variant.texture)?;
        let mut texture_directories = vec![texture_directory.clone()];
        if let Ok(path) = resolve_path(&texture_directory, "texture.cfg") {
            for fallback in cfg::texture::fallbacks(&config(&path)?)? {
                // Missing installed-simulator resources stay unresolved until requested.
                if let Ok(path) = resolve_path(&texture_directory, fallback.path)
                    && !texture_directories.contains(&path)
                {
                    texture_directories.push(path);
                }
            }
        }
        let datum = cfg::aircraft::ReferencePoints::from_config(&aircraft)?
            .datum_feet
            .unwrap_or([0.0; 3]);
        let eyepoint = cfg::camera::Views::from_config(&aircraft)?
            .eyepoint_feet
            .map(|[longitudinal, lateral, vertical]| {
                Vec3::new(
                    (lateral + datum[1]) as f32,
                    (vertical + datum[2]) as f32,
                    (longitudinal + datum[0]) as f32,
                ) * 0.3048
            });
        Ok(Self {
            title: variant.title.unwrap_or("Aircraft").into(),
            exterior,
            interior,
            eyepoint,
            texture_directories,
        })
    }

    pub fn texture_path(&self, name: &str) -> Result<Option<PathBuf>> {
        if name.starts_with('$') {
            return Ok(None);
        }
        for directory in &self.texture_directories {
            if let Ok(path) = resolve_path(directory, name) {
                return Ok(Some(path));
            }
            let alternative =
                Path::new(name).with_extension(if name.to_ascii_lowercase().ends_with(".dds") {
                    "bmp"
                } else {
                    "dds"
                });
            if let Ok(path) = resolve_path(directory, &alternative.to_string_lossy()) {
                return Ok(Some(path));
            }
        }
        bail!("missing aircraft texture {name}")
    }
}

impl Model {
    pub fn load(path: &Path) -> Result<Self> {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        Self::from_bytes(&bytes).with_context(|| format!("loading {}", path.display()))
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
        let decoded = mdl::Model::parse(bytes)?.decode()?;
        let data = decoded.exterior.context("model has no MDLD data")?;
        let parents = mdl::scene::parents(&data.nodes)?;
        let animations = animation::animations(&data)?;
        let visibility = animation::visibility(&data)?;
        let track_ids = animations
            .iter()
            .flat_map(|a| &a.tracks)
            .map(|t| t.id as usize)
            .collect::<HashSet<_>>();
        let links = |entries: &[i16], available: &HashSet<usize>| -> Result<Vec<Option<usize>>> {
            if entries.is_empty() {
                return Ok(vec![None; data.nodes.len()]);
            }
            anyhow::ensure!(
                entries.len() == data.nodes.len(),
                "node linkage count mismatch"
            );
            entries
                .iter()
                .map(|&entry| match entry {
                    -1 => Ok(None),
                    entry if entry >= 0 && available.contains(&(entry as usize)) => {
                        Ok(Some(entry as usize))
                    }
                    _ => bail!("invalid node linkage {entry}"),
                })
                .collect()
        };
        let animation_links = links(&data.animation_links, &track_ids)?;
        let visibility_links = links(&data.visibility_links, &(0..visibility.len()).collect())?;
        let nodes = parents
            .into_iter()
            .enumerate()
            .map(|(i, parent)| {
                let transform = Mat4::from_cols_array(data.node_transform(i as i32)?);
                if !transform.is_finite() || transform.determinant().abs() < 1e-12 {
                    bail!("invalid node transform {i}");
                }
                Ok(Node {
                    parent,
                    transform,
                    animation: animation_links[i],
                    visibility: visibility_links[i],
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let materials = data
            .materials
            .iter()
            .map(|material| {
                let maps = material.texture_maps(&data.textures)?;
                Ok(Material {
                    source: material.clone(),
                    diffuse_texture: maps
                        .diffuse
                        .map(|name| std::str::from_utf8(name).map(str::to_owned))
                        .transpose()?,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let mut lods = Vec::new();
        for lod in &data.lods {
            let mut meshes = Vec::new();
            for part in &lod.parts {
                if part.primitive_type != 1 {
                    bail!("unsupported mesh primitive {}", part.primitive_type);
                }
                let geometry = data.part_geometry(part)?;
                if !geometry.indices.len().is_multiple_of(3) {
                    bail!("incomplete triangle list");
                }
                if geometry.vertices.iter().any(|v| {
                    v.position
                        .iter()
                        .chain(&v.normal)
                        .chain(&v.uv)
                        .any(|v| !v.is_finite())
                }) {
                    bail!("non-finite vertex");
                }
                meshes.push(Mesh {
                    vertices: geometry.vertices.to_vec(),
                    indices: geometry.indices.to_vec(),
                    node: part.scene_node as usize,
                    material: part.material as usize,
                    mouse_rectangle: part.mouse_rectangle,
                    skin_weights: geometry.skin_weights.map(<[_]>::to_vec),
                    bones: data.bone_nodes(part.vertex_buffer)?,
                });
            }
            lods.push(Lod {
                level: lod.level,
                meshes,
            });
        }
        lods.sort_by_key(|lod| std::cmp::Reverse(lod.level));
        Ok(Self {
            nodes,
            materials,
            lods,
            animations,
            visibility,
        })
    }

    pub fn transforms(&self) -> Result<Vec<Mat4>> {
        let mut result = vec![Mat4::IDENTITY; self.nodes.len()];
        let mut ready = vec![false; self.nodes.len()];
        let mut chain = Vec::new();
        for index in 0..self.nodes.len() {
            let mut current = Some(index);
            while let Some(i) = current {
                let node = self.nodes.get(i).context("invalid parent node")?;
                if ready[i] {
                    break;
                }
                if chain.len() >= self.nodes.len() {
                    bail!("cyclic node hierarchy");
                }
                chain.push(i);
                current = node.parent;
            }
            while let Some(i) = chain.pop() {
                result[i] = self.nodes[i].parent.map_or(Mat4::IDENTITY, |p| result[p])
                    * self.nodes[i].transform;
                ready[i] = true;
            }
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_windows_paths_case_insensitively() {
        let temp = tempfile::tempdir().unwrap();
        std::fs::create_dir(temp.path().join("Texture")).unwrap();
        std::fs::write(temp.path().join("Texture/Test.DDS"), []).unwrap();
        assert!(
            resolve_path(temp.path(), "texture\\TEST.dds")
                .unwrap()
                .is_file()
        );
        assert!(resolve_path(temp.path(), "texture\\missing.dds").is_err());
    }

    #[test]
    fn composes_out_of_order_node_hierarchy() {
        let mut model = Model {
            nodes: vec![
                Node {
                    parent: Some(1),
                    transform: Mat4::from_translation(Vec3::X),
                    animation: None,
                    visibility: None,
                },
                Node {
                    parent: None,
                    transform: Mat4::from_rotation_y(std::f32::consts::FRAC_PI_2),
                    animation: None,
                    visibility: None,
                },
            ],
            materials: vec![],
            lods: vec![],
            animations: vec![],
            visibility: vec![],
        };
        assert!(
            model.transforms().unwrap()[0]
                .transform_point3(Vec3::ZERO)
                .abs_diff_eq(-Vec3::Z, 1e-6)
        );
        assert!(Model::from_bytes(b"invalid").is_err());
        model.nodes[1].parent = Some(0);
        assert!(model.transforms().is_err());
        model.nodes[1].parent = Some(2);
        assert!(model.transforms().is_err());
    }
}
