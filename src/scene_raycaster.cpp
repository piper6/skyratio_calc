#include "scene_raycaster.hpp"
#include <algorithm> // dont delete
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#define TINYBVH_IMPLEMENTATION
#include "ext/tinybvh/tiny_bvh.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
// 回転行列を生成(オイラー角から)
std::array<std::array<double, 3>, 3> euler_to_rotation_matrix(const Vec3& euler) {
  double cx = std::cos(euler[0]), sx = std::sin(euler[0]);
  double cy = std::cos(euler[1]), sy = std::sin(euler[1]);
  double cz = std::cos(euler[2]), sz = std::sin(euler[2]);

  return {{{cy * cz, -cy * sz, sy}, {sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy}, {-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy}}};
}

// ベクトルと行列の積
Vec3 matrix_vector_multiply(const std::array<std::array<double, 3>, 3>& mat, const Vec3& vec) { return {mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2], mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2], mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2]}; }

// UV球メッシュを生成
void generate_uv_sphere(const Vec3& center, double radius, int segments, int rings, std::vector<Vec3>& vertices, std::vector<Vec3i>& indices) {
  size_t base_idx = vertices.size();

  // 頂点を生成
  for(int ring = 0; ring <= rings; ring++) {
    double phi     = M_PI * ring / rings;
    double sin_phi = std::sin(phi);
    double cos_phi = std::cos(phi);

    for(int seg = 0; seg <= segments; seg++) {
      double theta     = 2.0 * M_PI * seg / segments;
      double sin_theta = std::sin(theta);
      double cos_theta = std::cos(theta);

      double x = radius * sin_phi * cos_theta;
      double y = radius * cos_phi;
      double z = radius * sin_phi * sin_theta;

      vertices.push_back({center[0] + x, center[1] + y, center[2] + z});
    }
  }

  // インデックスを生成
  for(int ring = 0; ring < rings; ring++) {
    for(int seg = 0; seg < segments; seg++) {
      int current = static_cast<unsigned int>(base_idx + ring * (segments + 1) + seg);
      int next    = current + segments + 1;
      indices.push_back(Vec3i{current, next, current + 1});
      indices.push_back(Vec3i{current + 1, next, next + 1});
    }
  }
}
} // namespace

SceneRaycaster::~SceneRaycaster() { delete bvh; }

SceneRaycaster::SceneRaycaster(SceneRaycaster&& other) noexcept : build_dirty(other.build_dirty), boxes(std::move(other.boxes)), spheres(std::move(other.spheres)), bvh(other.bvh), vertices(std::move(other.vertices)), indices(std::move(other.indices)) { other.bvh = nullptr; }

SceneRaycaster& SceneRaycaster::operator=(SceneRaycaster&& other) noexcept {
  if(this != &other) {
    delete bvh;

    build_dirty = other.build_dirty;
    boxes       = std::move(other.boxes);
    spheres     = std::move(other.spheres);
    vertices    = std::move(other.vertices);
    indices     = std::move(other.indices);
    bvh         = other.bvh;
    other.bvh   = nullptr;
  }
  return *this;
}

void SceneRaycaster::clear() {
  boxes.clear();
  spheres.clear();
  vertices.clear();
  indices.clear();
  delete bvh;
  bvh         = nullptr;
  build_dirty = true;
}

void SceneRaycaster::add_box(const Vec3& pos, const Vec3& size, const Vec3& euler) {
  boxes.push_back({pos, size, euler});
  build_dirty = true;
}

void SceneRaycaster::add_sphere(const Vec3& center, double radius) {
  spheres.push_back({center, radius});
  build_dirty = true;
}

void SceneRaycaster::add_mesh(const std::vector<Vec3>& mesh_vertices) {
  if(mesh_vertices.size() % 3 != 0) {
    return;
  }

  size_t base_idx = vertices.size() / 3;

  for(const auto& v : mesh_vertices) vertices.push_back(v);

  for(size_t i = 0; i < mesh_vertices.size() / 3; i++) //
    indices.push_back(Vec3i{(int)(base_idx + i * 3), (int)(base_idx + i * 3 + 1), (int)(base_idx + i * 3 + 2)});
  build_dirty = true;
}

void SceneRaycaster::build() {
  // ボックスをメッシュに変換
  for(const auto& box : boxes) {
    auto rot_mat   = euler_to_rotation_matrix(box.euler);
    Vec3 half_size = {box.size[0] / 2, box.size[1] / 2, box.size[2] / 2};

    std::vector<Vec3> corners(8);
    for(int i = 0; i < 8; i++) {
      Vec3 local   = {(i & 1) ? half_size[0] : -half_size[0], (i & 2) ? half_size[1] : -half_size[1], (i & 4) ? half_size[2] : -half_size[2]};
        Vec3 rotated = matrix_vector_multiply(rot_mat, local);
      corners[i]   = {rotated[0] + box.center[0], rotated[1] + box.center[1], rotated[2] + box.center[2]};
    }

    std::vector<std::array<int, 3>> faces = {{0, 1, 2}, {2, 1, 3}, {4, 6, 5}, {5, 6, 7}, {0, 2, 4}, {4, 2, 6}, {1, 5, 3}, {3, 5, 7}, {0, 4, 1}, {1, 4, 5}, {2, 3, 6}, {6, 3, 7}};

    size_t base_idx = vertices.size();
    for(const auto& c : corners) vertices.push_back(c);
    for(const auto& f : faces) {
      indices.push_back(Vec3i{(int)(base_idx + f[0]), (int)(base_idx + f[1]), (int)(base_idx + f[2])});
    }
  }

  // 球をUV球メッシュに変換
  for(const auto& sphere : spheres) {
    generate_uv_sphere(sphere.center, sphere.radius, 16, 8, vertices, indices);
  }

  // BVHを構築
  if(!vertices.empty() && !indices.empty()) {
    triangles.clear();
    triangles.reserve(indices.size());
    for(size_t i = 0; i < indices.size(); i++) {
      for(int k = 0; k < 3; k++) {
        const auto& v = vertices[indices[i][k]];
        triangles.push_back(tinybvh::bvhvec4(v[0], v[1], v[2], 0.0f));
      }
    }

    delete bvh;
    bvh = new tinybvh::BVH();
    bvh->Build(triangles.data(), triangles.size() / 3);
  }
  build_dirty = false;
}

std::vector<HitResult> SceneRaycaster::raycast(const std::vector<Vec3>& origins, const std::vector<Vec3>& directions) const {
  std::vector<HitResult> results(origins.size());

  if(!bvh || origins.empty()) {
    printf("BVH = %p, num rays = %zu\n", (void*)bvh, origins.size());
    throw std::runtime_error("BVH is not built or no rays to cast.");
    return results;
  }

  std::vector<tinybvh::Ray> rays(origins.size());
  for(size_t i = 0; i < origins.size(); i++) {
    rays[i] = tinybvh::Ray(                                                                       //
      tinybvh::bvhvec3((float)origins[i][0], (float)origins[i][1], (float)origins[i][2]),         //
      tinybvh::bvhvec3((float)directions[i][0], (float)directions[i][1], (float)directions[i][2]) //
    );
    // rays[i].hit.inst = std::numeric_limits<uint32_t>::max();
    // rays[i].hit.prim = std::numeric_limits<uint32_t>::max();
    // rays[i].hit.t    = 1e30f; // 初期化
  }
#if 1
  // 256レイずつバッチ処理
  const size_t batch_size  = 256;
  const size_t num_batches = (rays.size() + batch_size - 1) / batch_size;

  for(size_t batch = 0; batch < num_batches; batch++) {
    size_t start = batch * batch_size;
    size_t end   = std::min(start + batch_size, rays.size());
    size_t count = end - start;

    // if(count == batch_size) {
    //   // フルバッチの場合は最適化版を使用
    //   bvh->Intersect256Rays(&rays[start]);
    // } else {
    //   // 部分バッチの場合は個別に処理
    //   for(size_t i = start; i < end; i++) {
    //     bvh->Intersect(rays[i]);
    //   }
    // }
    //printf("Processing batch %zu/%zu (rays %zu to %zu)\n", batch + 1, num_batches, start, end);
    for(size_t i = start; i < end; i++) {
        bvh->Intersect(rays[i]);
    }
  }

  // 結果を変換
  for(size_t i = 0; i < rays.size(); i++) {
    if(rays[i].hit.t < 1e30f) {
      results[i].hit         = true;
      results[i].distance    = rays[i].hit.t;
      results[i].position[0] = origins[i][0] + directions[i][0] * rays[i].hit.t;
      results[i].position[1] = origins[i][1] + directions[i][1] * rays[i].hit.t;
      results[i].position[2] = origins[i][2] + directions[i][2] * rays[i].hit.t;
    } else {
      results[i].hit      = false;
      results[i].distance = std::numeric_limits<double>::infinity();
    }
  }
#else
  for(size_t i = 0; i < rays.size(); i++) results[i].hit = bvh->IsOccluded(rays[i]);
#endif

  return results;
}

void SceneRaycaster::save(const char* filepath) {
  std::ofstream file(filepath, std::ios::binary);
  if(!file) {
    throw std::runtime_error(std::string("Failed to open file for writing: ") + filepath);
  }

  // STL binary format header (80 bytes)
  char header[80] = {0};
  file.write(header, 80);

  // Number of triangles (4 bytes)
  uint32_t num_triangles = static_cast<uint32_t>(indices.size());
  file.write(reinterpret_cast<const char*>(&num_triangles), sizeof(uint32_t));

  // Write each triangle
  for(const auto& idx : indices) {
    // Validate indices
    if(idx[0] >= static_cast<int>(vertices.size()) || idx[1] >= static_cast<int>(vertices.size()) || idx[2] >= static_cast<int>(vertices.size())) {
      throw std::runtime_error("Invalid vertex index in triangle");
    }

    // Calculate normal vector (cross product)
    const Vec3& v0 = vertices[idx[0]];
    const Vec3& v1 = vertices[idx[1]];
    const Vec3& v2 = vertices[idx[2]];

    Vec3 edge1 = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
    Vec3 edge2 = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
    Vec3 normal = {edge1[1] * edge2[2] - edge1[2] * edge2[1], edge1[2] * edge2[0] - edge1[0] * edge2[2], edge1[0] * edge2[1] - edge1[1] * edge2[0]};

    // Normalize
    double len = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if(len > 0.0) {
      normal[0] /= len;
      normal[1] /= len;
      normal[2] /= len;
    } else {
      // Degenerate triangle: use default normal
      normal = {0.0, 0.0, 1.0};
    }

    // Write normal
    float normal_f[3] = {static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2])};
    file.write(reinterpret_cast<const char*>(normal_f), 3 * sizeof(float));

    // Write vertices
    for(int i = 0; i < 3; i++) {
      const Vec3& v = vertices[idx[i]];
      float v_f[3]  = {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
      file.write(reinterpret_cast<const char*>(v_f), 3 * sizeof(float));
    }

    // Attribute byte count (2 bytes, typically 0)
    uint16_t attribute_count = 0;
    file.write(reinterpret_cast<const char*>(&attribute_count), sizeof(uint16_t));
  }

  file.close();
}
