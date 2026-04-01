#include "sky_ratio_checker.hpp"
#include <cassert>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 天頂角の範囲設定（レイキャストの負荷軽減のため）
constexpr double THETA_MIN_DEG = 0.0;
constexpr double THETA_MAX_DEG = 89.0;

namespace {
constexpr int FIXED_NUM_RAYS = 1000;
} // namespace

std::vector<std::tuple<Vec3, Vec3>> SkyRatioChecker::generate_rays_from_checkpoint(const Vec3& checkpoint) {
  /*
  if(ray_resolution <= 0.0f || ray_resolution > 180.0f) ray_resolution = 1.0f;
  // 天頂角(theta): 20度から89度までに変更（負荷軽減のため）
  int theta_steps = static_cast<int>((THETA_MAX_DEG - THETA_MIN_DEG) / ray_resolution);

  // 方位角(phi): 0度から360度まで
  int phi_steps = static_cast<int>(360.0 / ray_resolution);

  if(phi_steps < 1) phi_steps = 1;

  std::vector<std::tuple<Vec3, Vec3>> rays;
  for(int t = 0; t <= theta_steps; t++) {
    double theta     = (THETA_MIN_DEG + t * ray_resolution) * M_PI / 180.0;
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);

    for(int p = 0; p < phi_steps; p++) {
      double phi = p * 2.0 * M_PI / phi_steps;

      // 球面座標から直交座標への変換
      Vec3 direction{cos_theta * std::cos(phi), cos_theta * std::sin(phi), sin_theta};
      rays.push_back(std::make_tuple(checkpoint, direction));
    }
  }

  return rays;
  */

  const int num_rays            = FIXED_NUM_RAYS;
  const double golden_angle_rad = M_PI * (3.0 - std::sqrt(5.0));

  std::vector<std::tuple<Vec3, Vec3>> rays;
  rays.reserve(num_rays);

  if(num_rays == 1) {
    rays.push_back(std::make_tuple(checkpoint, Vec3{0.0, 0.0, 1.0}));
    return rays;
  }

  for(int i = 0; i < num_rays; i++) {
    const double z      = 1.0 - (static_cast<double>(i) / static_cast<double>(num_rays - 1));
    const double radius = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double theta  = golden_angle_rad * static_cast<double>(i);

    Vec3 direction{
      std::cos(theta) * radius,
      std::sin(theta) * radius,
      z,
    };
    rays.push_back(std::make_tuple(checkpoint, direction));
  }

  return rays;
}

std::vector<float> SkyRatioChecker::check(SceneRaycaster* raycaster) {
  /*
  if(raycaster == nullptr) {
    printf("[ERROR] SkyRatioChecker: SceneRaycaster is not set.\n");
    return {};
  }

  std::vector<float> results;
  results.reserve(checkpoints.size());

  raycaster->build();
  if(raycaster->vertices.empty() || raycaster->indices.empty()) {
    printf("[WARNING] SkyRatioChecker: SceneRaycaster has no geometry.\n");
    return {1};
  }

  for(const auto& checkpoint : checkpoints) {
    auto rays = generate_rays_from_checkpoint(checkpoint);

    // レイの原点と方向を分離
    std::vector<Vec3> origins, directions;
    origins.reserve(rays.size());
    directions.reserve(rays.size());

    for(const auto& ray : rays) {
      origins.push_back(std::get<0>(ray));
      directions.push_back(std::get<1>(ray));
    }

    const auto hit_results    = raycaster->raycast(origins, directions);
    const auto resolution_rad = ray_resolution * M_PI / 180.0;
    const int phi_steps       = std::max((int)(360.0 / ray_resolution), 1);
    const int theta_steps     = static_cast<int>((THETA_MAX_DEG - THETA_MIN_DEG) / ray_resolution);

    if((theta_steps + 1) * phi_steps != (int)hit_results.size()) {
      printf("[ERROR] SkyRatioChecker: Raycast result size mismatch. Expected %d, got %zu\n", //
             (theta_steps + 1) * phi_steps, hit_results.size());
      results.push_back(-1.0f);
      continue;
    }

    // 各方位角(phi)における、空が見える最小天頂角を格納する配列
    std::vector<double> visible_theta(phi_steps, 0.0);

    for(int p = 0; p < phi_steps; p++) {
      // この方位角での最大遮蔽角度（空が見え始める角度）
      double max_blocked_theta = 0.0;

      for(int t = 0; t <= theta_steps; t++) {
        int ray_index = t * phi_steps + p;
        if(!hit_results[ray_index].hit) continue;
        // 建物にヒット → この角度では空が見えない
        double blocked_theta = (THETA_MIN_DEG + ray_resolution * t) * M_PI / 180.0;

        // 安全側評価の適用
        if(use_safe_side) {
          blocked_theta += resolution_rad; // 内接近似：建物を大きく見積もる（空を小さく見積もる）
        } else {
          blocked_theta -= resolution_rad; // 外接近似：建物を小さく見積もる（空を大きく見積もる）
        }
        if(blocked_theta > M_PI / 2.0) blocked_theta = M_PI / 2.0;
        max_blocked_theta = std::max(max_blocked_theta, blocked_theta);
      }
      visible_theta[p] = max_blocked_theta;
    }

    // 三斜求積法による面積計算
    double sky_area = 0.0;
    for(int p = 0; p < phi_steps; p++) {
      // 2つの隣接する方位角での空が見える開始角度（遮蔽終了角度）
      double theta1       = visible_theta[p];
      double theta2       = visible_theta[(p + 1) % phi_steps];
      double segment_area = std::cos(theta1) * std::cos(theta2);
      sky_area += segment_area;
    }

    float sky_ratio = sky_area / phi_steps;

    // 範囲制限
    if(sky_ratio < 0.0f) sky_ratio = 0.0f;
    if(sky_ratio > 1.0f) sky_ratio = 1.0f;

    results.push_back(sky_ratio);
  }

  return results;
  */

  if(raycaster == nullptr) {
    printf("[ERROR] SkyRatioChecker: SceneRaycaster is not set.\n");
    return {};
  }

  std::vector<float> results;
  results.reserve(checkpoints.size());

  raycaster->build();
  if(raycaster->vertices.empty() || raycaster->indices.empty()) {
    printf("[WARNING] SkyRatioChecker: SceneRaycaster has no geometry.\n");
    return {1};
  }

  for(const auto& checkpoint : checkpoints) {
    auto rays = generate_rays_from_checkpoint(checkpoint);

    std::vector<Vec3> origins, directions;
    origins.reserve(rays.size());
    directions.reserve(rays.size());

    for(const auto& ray : rays) {
      origins.push_back(std::get<0>(ray));
      directions.push_back(std::get<1>(ray));
    }

    const auto hit_results = raycaster->raycast(origins, directions);
    if(hit_results.size() != directions.size()) {
      printf("[ERROR] SkyRatioChecker: Raycast result size mismatch. Expected %zu, got %zu\n",
             directions.size(),
             hit_results.size());
      results.push_back(-1.0f);
      continue;
    }

    double total_weight      = 0.0;
    double obstructed_weight = 0.0;
    for(size_t i = 0; i < directions.size(); i++) {
      const double weight = directions[i][2];
      total_weight += weight;
      if(hit_results[i].hit) obstructed_weight += weight;
    }

    if(total_weight <= 0.0) total_weight = 1.0;

    float sky_ratio = static_cast<float>(1.0 - (obstructed_weight / total_weight));
    if(sky_ratio < 0.0f) sky_ratio = 0.0f;
    if(sky_ratio > 1.0f) sky_ratio = 1.0f;

    results.push_back(sky_ratio);
  }

  // return results;
  return  results;
}
