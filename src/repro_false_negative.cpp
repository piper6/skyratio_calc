#include "scene_raycaster.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

struct BoxPayload {
  Vec3 center;
  Vec3 size;
  Vec3 euler;
};

BoxPayload serialize_box_like_worker(const Vec3& position, const Vec3& size, double yaw) {
  return {
    {
      position[0] + size[0] * 0.5,
      position[1] + size[1] * 0.5,
      position[2] + size[2] * 0.5,
    },
    size,
    {0.0, 0.0, yaw},
  };
}

SceneRaycaster build_scene_like_worker(const BoxPayload& payload) {
  SceneRaycaster scene;
  scene.add_box(payload.center, payload.size, payload.euler);
  scene.build(); // worker.py::_build_scene_from_payload と同じ
  return scene;
}

bool analytic_ray_box_hit(const Vec3& origin, const Vec3& direction, const Vec3& box_center, const Vec3& box_size) {
  const Vec3 box_min = {
    box_center[0] - box_size[0] * 0.5,
    box_center[1] - box_size[1] * 0.5,
    box_center[2] - box_size[2] * 0.5,
  };
  const Vec3 box_max = {
    box_center[0] + box_size[0] * 0.5,
    box_center[1] + box_size[1] * 0.5,
    box_center[2] + box_size[2] * 0.5,
  };

  double t_min = -INFINITY;
  double t_max = INFINITY;

  for(int axis = 0; axis < 3; axis++) {
    if(std::abs(direction[axis]) < 1e-12) {
      if(origin[axis] < box_min[axis] || origin[axis] > box_max[axis]) return false;
      continue;
    }

    double t1 = (box_min[axis] - origin[axis]) / direction[axis];
    double t2 = (box_max[axis] - origin[axis]) / direction[axis];
    if(t1 > t2) std::swap(t1, t2);

    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if(t_min > t_max) return false;
  }

  return t_max > 1e-9;
}

void print_hit_summary(const char* label, const HitResult& hit) {
  std::cout << label << ": hit=" << std::boolalpha << hit.hit << " distance=" << std::setprecision(12) << hit.distance;
  if(hit.hit) {
    std::cout << " position=(" << hit.position[0] << ", " << hit.position[1] << ", " << hit.position[2] << ")";
  }
  std::cout << '\n';
}

} // namespace

int main() {
  const Vec3 origin = {0.0, -2.0, 0.0};
  const Vec3 direction = {0.044290756, 0.141205909, 0.988988989}; // ray=0011

  // compare_new_old2.py の現在設定
  const Vec3 box_position = {0.01, 0.0, 0.0};
  const Vec3 box_size = {16.5, 24.0, 32.0};
  const double box_yaw = 0.0;

  // worker.py::_serialize_boxes と同じ変換
  const BoxPayload payload = serialize_box_like_worker(box_position, box_size, box_yaw);

  const bool expected_hit = analytic_ray_box_hit(origin, direction, payload.center, payload.size);

  SceneRaycaster scene = build_scene_like_worker(payload);

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "origin=(" << origin[0] << ", " << origin[1] << ", " << origin[2] << ")\n";
  std::cout << "direction=(" << direction[0] << ", " << direction[1] << ", " << direction[2] << ")\n";
  std::cout << "box_position=(" << box_position[0] << ", " << box_position[1] << ", " << box_position[2] << ")\n";
  std::cout << "box_size=(" << box_size[0] << ", " << box_size[1] << ", " << box_size[2] << ")\n";
  std::cout << "box_yaw=" << box_yaw << "\n";
  std::cout << "serialized_center=(" << payload.center[0] << ", " << payload.center[1] << ", " << payload.center[2] << ")\n";
  std::cout << "serialized_euler=(" << payload.euler[0] << ", " << payload.euler[1] << ", " << payload.euler[2] << ")\n";
  std::cout << "analytic_hit=" << std::boolalpha << expected_hit << "\n";

  // worker.py で build 済みの scene に対して raycast するのと同じ
  {
    std::vector<Vec3> origins = {origin};
    std::vector<Vec3> directions = {direction};
    const auto hits = scene.raycast(origins, directions);
    print_hit_summary("worker-like scene.raycast(1 ray)", hits[0]);
  }

  // 実運用では checker.check() 内で build() がもう一度呼ばれるので、その条件も再現
  scene.build();
  {
    std::vector<Vec3> origins = {origin};
    std::vector<Vec3> directions = {direction};
    const auto hits = scene.raycast(origins, directions);
    print_hit_summary("after second build scene.raycast(1 ray)", hits[0]);
  }

  // Force the 256-ray batch path with the same ray duplicated.
  {
    std::vector<Vec3> origins(256, origin);
    std::vector<Vec3> directions(256, direction);
    const auto hits = scene.raycast(origins, directions);
    print_hit_summary("after second build scene.raycast(256 duplicated rays) first", hits[0]);
  }

  return 0;
}
