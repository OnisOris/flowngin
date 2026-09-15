use rapier3d::prelude::{ColliderBuilder, TriMeshFlags, Vector};
use std::fs;
use std::io;
use std::path::Path;

/// Длина стороны квадратной карты (в единицах мира).
pub const MAP_SIZE: f32 = 80.0;
/// Число ячеек сетки на одну сторону карты.
pub const GRID_RES: usize = 80;
/// Шаг сетки в единицах мира.
const STEP: f32 = MAP_SIZE / GRID_RES as f32;
/// Половина стороны карты, карта центрирована в начале координат.
const HALF: f32 = MAP_SIZE * 0.5;

/// Рельеф пола: регулярная сетка высот и готовый треугольный меш.
pub struct Terrain {
    /// Высоты в узлах сетки: `j*(GRID_RES+1) + i`, где `i` — по X, `j` — по Z.
    pub heights: Vec<f32>,
    /// Вершины меша (одна на узел сетки, порядок совпадает с `heights`).
    pub vertices: Vec<Vector>,
    /// Индексы треугольников (два на ячейку сетки).
    pub indices: Vec<[u32; 3]>,
}

impl Terrain {
    /// Строит рельеф: умеренный шум на части карты и большая гора в центре.
    pub fn generate() -> Self {
        let n = GRID_RES + 1;
        let mut heights = Vec::with_capacity(n * n);
        let mut vertices = Vec::with_capacity(n * n);
        for j in 0..n {
            let z = -HALF + j as f32 * STEP;
            for i in 0..n {
                let x = -HALF + i as f32 * STEP;
                let h = terrain_height(x, z);
                heights.push(h);
                vertices.push(Vector::new(x, h, z));
            }
        }

        // Два треугольника на каждую ячейку: (a,c,b) и (b,c,d).
        let mut indices = Vec::with_capacity(GRID_RES * GRID_RES * 2);
        for j in 0..GRID_RES {
            for i in 0..GRID_RES {
                let a = j * n + i;
                let b = j * n + i + 1;
                let c = (j + 1) * n + i;
                let d = (j + 1) * n + i + 1;
                indices.push([a as u32, c as u32, b as u32]);
                indices.push([b as u32, c as u32, d as u32]);
            }
        }

        Self {
            heights,
            vertices,
            indices,
        }
    }

    /// Высота рельефа в произвольной точке (билинейная интерполяция по сетке).
    pub fn height_at(&self, x: f32, z: f32) -> f32 {
        let u = ((x + HALF) / STEP).clamp(0.0, GRID_RES as f32);
        let v = ((z + HALF) / STEP).clamp(0.0, GRID_RES as f32);
        let i = u.floor() as usize;
        let j = v.floor() as usize;
        let i1 = (i + 1).min(GRID_RES);
        let j1 = (j + 1).min(GRID_RES);
        let fu = u - i as f32;
        let fv = v - j as f32;
        let n = GRID_RES + 1;
        let at = |rx: usize, rz: usize| self.heights[rz * n + rx];
        let a = at(i, j) + (at(i1, j) - at(i, j)) * fu;
        let b = at(i, j1) + (at(i1, j1) - at(i, j1)) * fu;
        a + (b - a) * fv
    }

    /// Создаёт коллайдер-тримеш для фиксированного тела пола.
    pub fn collider(&self) -> ColliderBuilder {
        ColliderBuilder::trimesh_with_flags(
            self.vertices.clone(),
            self.indices.clone(),
            TriMeshFlags::FIX_INTERNAL_EDGES,
        )
        .expect("меш пола должен содержать хотя бы один треугольник")
        .friction(1.0)
    }

    /// Сохраняет рельеф в бинарный STL-файл.
    pub fn write_stl(&self, path: &Path) -> io::Result<()> {
        let mut out: Vec<u8> = Vec::with_capacity(84 + 50 * self.indices.len());
        // 80 байт заголовка + счётчик треугольников.
        let mut header = [0u8; 80];
        header[..8].copy_from_slice(b"flowngin");
        out.extend_from_slice(&header);
        out.extend_from_slice(&(self.indices.len() as u32).to_le_bytes());
        // 50 байт на треугольник: нормаль, три вершины, атрибут.
        for tri in &self.indices {
            let a = self.vertices[tri[0] as usize];
            let b = self.vertices[tri[1] as usize];
            let c = self.vertices[tri[2] as usize];
            let n = (b - a).cross(c - a).normalize_or_zero();
            for v in [n.x, n.y, n.z] {
                out.extend_from_slice(&v.to_le_bytes());
            }
            for p in [a, b, c] {
                for v in [p.x, p.y, p.z] {
                    out.extend_from_slice(&v.to_le_bytes());
                }
            }
            out.extend_from_slice(&0u16.to_le_bytes());
        }
        fs::write(path, out)
    }

    /// Читает меш из бинарного STL-файла (вершины и нормали хранятся в файле).
    pub fn read_stl(path: &Path) -> io::Result<StlMesh> {
        let data = fs::read(path)?;
        if data.len() < 84 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "STL слишком короткий: нет заголовка и счётчика треугольников",
            ));
        }
        let count = u32::from_le_bytes(data[80..84].try_into().unwrap()) as usize;
        let mut buf = [0u8; 4];
        let mut read_f32 = |off: usize| -> io::Result<f32> {
            buf.copy_from_slice(&data[off..off + 4]);
            Ok(f32::from_le_bytes(buf))
        };
        let mut triangles = Vec::with_capacity(count);
        let mut normals = Vec::with_capacity(count);
        for i in 0..count {
            let off = 84 + i * 50;
            if off + 50 > data.len() {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "STL обрезан: не хватает байт на треугольник",
                ));
            }
            let n = Vector::new(read_f32(off)?, read_f32(off + 4)?, read_f32(off + 8)?);
            let a = Vector::new(
                read_f32(off + 12)?,
                read_f32(off + 16)?,
                read_f32(off + 20)?,
            );
            let b = Vector::new(
                read_f32(off + 24)?,
                read_f32(off + 28)?,
                read_f32(off + 32)?,
            );
            let c = Vector::new(
                read_f32(off + 36)?,
                read_f32(off + 40)?,
                read_f32(off + 44)?,
            );
            triangles.push([a, b, c]);
            normals.push(n);
        }
        Ok(StlMesh { triangles, normals })
    }

    /// Читает STL-меш и восстанавливает `Terrain` (вместе с сеткой высот для `height_at`).
    ///
    /// STL хранит только треугольники, поэтому сетка снова раскладывается по
    /// регулярным узлам `GRID_RES` — файл должен быть записан нашей `write_stl`.
    pub fn load_stl(path: &Path) -> Result<Terrain, String> {
        let mesh = Terrain::read_stl(path).map_err(|e| e.to_string())?;
        Terrain::from_stl(&mesh)
    }

    /// Восстанавливает `Terrain` из прочитанного STL-меша, раскладывая вершины
    /// по регулярной сетке `GRID_RES`.
    pub fn from_stl(mesh: &StlMesh) -> Result<Terrain, String> {
        let n = GRID_RES + 1;
        // Номер ячейки сетки по координате: x = -HALF + cell*STEP.
        let cell = |p: Vector| {
            let gi = ((p.x + HALF) / STEP + 0.5) as i32;
            let gj = ((p.z + HALF) / STEP + 0.5) as i32;
            let k = |v: i32| {
                if (0..n as i32).contains(&v) {
                    Some(v as usize)
                } else {
                    None
                }
            };
            (k(gi), k(gj))
        };

        // Проходим по всем треугольникам и собираем высоты в узлы сетки.
        let mut heights = vec![0.0f32; n * n];
        for tri in &mesh.triangles {
            for p in tri.iter() {
                let (Some(gi), Some(gj)) = cell(*p) else {
                    return Err(format!(
                        "вершина ({}, {}) вне сетки 0..{}",
                        p.x, p.z, GRID_RES
                    ));
                };
                heights[gj * n + gi] = p.y;
            }
        }

        // Вершины в каноническом порядке j*n+i (как в `generate`).
        let mut vertices = Vec::with_capacity(n * n);
        for gj in 0..n {
            for gi in 0..n {
                vertices.push(Vector::new(
                    -HALF + gi as f32 * STEP,
                    heights[gj * n + gi],
                    -HALF + gj as f32 * STEP,
                ));
            }
        }

        // Индексы из намотки файла, переведённые в канонические номера вершин.
        let mut indices = Vec::with_capacity(mesh.triangles.len());
        for tri in &mesh.triangles {
            let mut t = [0u32; 3];
            for (s, p) in tri.iter().enumerate() {
                let (Some(gi), Some(gj)) = cell(*p) else {
                    unreachable!("выше уже проверили, что клетка в сетке");
                };
                t[s] = (gj * n + gi) as u32;
            }
            indices.push(t);
        }

        Ok(Terrain {
            heights,
            vertices,
            indices,
        })
    }
}

/// Меш, прочитанный из бинарного STL-файла.
pub struct StlMesh {
    /// Три вершины каждого треугольника (между соседними треугольниками дублируются).
    pub triangles: Vec<[Vector; 3]>,
    /// Нормали, сохранённые в файле (одна на треугольник).
    pub normals: Vec<Vector>,
}

impl StlMesh {
    /// Коллайдер-тримеш из меша — если сетка высот (`Terrain`) не нужна.
    pub fn collider(&self) -> ColliderBuilder {
        let mut vertices = Vec::with_capacity(self.triangles.len() * 3);
        let mut indices = Vec::with_capacity(self.triangles.len());
        for (i, tri) in self.triangles.iter().enumerate() {
            let base = (i * 3) as u32;
            indices.push([base, base + 1, base + 2]);
            vertices.extend_from_slice(tri);
        }
        ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::FIX_INTERNAL_EDGES)
            .expect("меш пола должен содержать хотя бы один треугольник")
            .friction(1.0)
    }
}

/// Генеративная высота рельефа в точке (x, z).
///
/// - `mountain` — большая гора в центре карты (гаусс).
/// - `noise_zone` — плавная маска: слева почти равнина, справа умеренный шум.
fn terrain_height(x: f32, z: f32) -> f32 {
    let d2 = x * x + z * z;
    let mountain = 5.0 * (-d2 / (2.0 * 13.0 * 13.0)).exp();
    let noise_zone = 0.5 + 0.5 * (x * 0.06).tanh();
    let noise = fbm(x * 0.09, z * 0.09, 3, 2.0, 0.5) * (0.55 * noise_zone);
    mountain + noise
}

/// Детерминированный «стохастический» шум на основе хеша целочисленных узлов.
fn value_noise(x: f32, z: f32) -> f32 {
    let ix = x.floor() as i32;
    let iz = z.floor() as i32;
    let fx = x - x.floor();
    let fz = z - z.floor();
    let sx = fx * fx * (3.0 - 2.0 * fx);
    let sz = fz * fz * (3.0 - 2.0 * fz);
    let v00 = hash2(ix, iz);
    let v10 = hash2(ix + 1, iz);
    let v01 = hash2(ix, iz + 1);
    let v11 = hash2(ix + 1, iz + 1);
    let a0 = v00 + (v10 - v00) * sx;
    let a1 = v01 + (v11 - v01) * sx;
    a0 + (a1 - a0) * sz
}

/// Сумма нескольких октав шума, результатов примерно в диапазоне [-1, 1].
fn fbm(x: f32, z: f32, octaves: u32, lacunarity: f32, gain: f32) -> f32 {
    let mut amp = 1.0;
    let mut freq = 1.0;
    let mut sum = 0.0;
    let mut norm = 0.0;
    for _ in 0..octaves {
        sum += value_noise(x * freq, z * freq) * amp;
        norm += amp;
        amp *= gain;
        freq *= lacunarity;
    }
    sum / norm
}

/// Хеш двух целых координат в псевдослучайное число из [-1, 1].
fn hash2(ix: i32, iz: i32) -> f32 {
    let mut n = (ix as u32).wrapping_mul(3_747_613) ^ (iz as u32).wrapping_mul(6_682_652);
    n = n.wrapping_mul(n ^ (n >> 16));
    n ^= n >> 13;
    n = n.wrapping_mul(1_274_126_173);
    n ^= n >> 16;
    ((n & 0xFFFFFF) as f32) / 0xFFFFFF as f32 * 2.0 - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn stl_roundtrip() {
        let terrain = Terrain::generate();
        let path = std::env::temp_dir().join("flowngin_roundtrip.stl");
        terrain.write_stl(&path).unwrap();
        let mesh = Terrain::read_stl(&path).unwrap();
        fs::remove_file(path).ok();

        assert_eq!(mesh.triangles.len(), terrain.indices.len());
        assert_eq!(mesh.normals.len(), terrain.indices.len());
        for (tri, &idx) in mesh.triangles.iter().zip(&terrain.indices) {
            let a = terrain.vertices[idx[0] as usize];
            let b = terrain.vertices[idx[1] as usize];
            let c = terrain.vertices[idx[2] as usize];
            assert_eq!(tri[0], a);
            assert_eq!(tri[1], b);
            assert_eq!(tri[2], c);
        }
        // Сохранённая нормаль совпадает с намоткой и смотрит вверх.
        for (tri, &stored) in mesh.triangles.iter().zip(&mesh.normals) {
            let from_winding = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize_or_zero();
            assert!(from_winding.y > 0.0, "намотка должна давать нормали вверх");
            assert!(stored.distance(from_winding) < 1e-3);
        }

        // Восстановленный Terrain должен совпасть с исходным.
        let restored = Terrain::from_stl(&mesh).unwrap();
        assert_eq!(restored.heights, terrain.heights);
        assert_eq!(restored.vertices, terrain.vertices);
        assert_eq!(restored.indices, terrain.indices);
    }
}
