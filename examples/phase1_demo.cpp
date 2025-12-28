#include <iostream>

#include "../include/frames.hpp"
#include "../include/units.hpp"

using namespace sensor_pipeline;
using namespace sensor_pipeline::literals;

int main() {
  std::cout << "=== ADAS Sensor Pipeline - Phase 1 Demo ===\n\n";

  // ========================================================================
  // 1. Type-Safe Units
  // ========================================================================
  std::cout << "--- Type-Safe Units ---\n";

  auto distance = 150.5_m;
  auto time = 5.0_s;

  std::cout << "Distance: " << distance << "\n";
  std::cout << "Time: " << time << "\n";

  // Unit conversions
  auto dist_km = convert<Kilometers>(distance);
  auto dist_mm = convert<Millimeters>(distance);

  std::cout << "Distance in km: " << dist_km << "\n";
  std::cout << "Distance in mm: " << dist_mm << "\n";

  // Velocity units
  auto speed_mps = 30.0_mps;
  auto speed_kph = 108.0_kph;
  auto speed_kph_converted = convert<KilometersPerHour>(speed_mps);

  std::cout << "Speed: " << speed_mps << "\n";
  std::cout << "Speed: " << speed_kph << "\n";
  std::cout << "30 m/s in km/h: " << speed_kph_converted << "\n";

  // ⚠️ Compile-time error prevention:
  // auto wrong = distance + time;  // ❌ Won't compile! Different dimensions
  // auto wrong2 = distance + speed_mps; // ❌ Won't compile! Incompatible types

  std::cout << "\n";

  // ========================================================================
  // 2. Coordinate Frames
  // ========================================================================
  std::cout << "--- Coordinate Frames ---\n";

  // Define points in different frames
  const Point3D<LidarFrame> lidar_point{10.0_m, 5.0_m, 2.0_m};
  const Point3D<VehicleFrame> vehicle_point{15.0_m, 0.0_m, 1.5_m};

  std::cout << "Lidar point: (" << lidar_point.x << ", " << lidar_point.y
            << ", " << lidar_point.z << ")\n";

  std::cout << "Vehicle point: (" << vehicle_point.x << ", " << vehicle_point.y
            << ", " << vehicle_point.z << ")\n";

  // ⚠️ Compile-time frame checking:
  // auto wrong_add = lidar_point.x + vehicle_point.x; // ✓ Same unit type
  // However, semantic mixing is prevented by type system

  std::cout << "\n";

  // ========================================================================
  // 3. Frame Transforms
  // ========================================================================
  std::cout << "--- Frame Transforms ---\n";

  // Define transform from Lidar to Vehicle frame
  // Lidar is mounted 2m in front, 0m lateral, 1.5m up from vehicle origin
  const Transform<LidarFrame, VehicleFrame> T_lidar_to_vehicle{
      Point3D<VehicleFrame>(2.0_m, 0.0_m, 1.5_m),  // Translation
      {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}  // Identity rotation (aligned)
  };

  // Transform point from Lidar to Vehicle frame
  const Point3D<LidarFrame> obstacle_lidar{8.0_m, 2.0_m, 0.0_m};
  const Point3D<VehicleFrame> obstacle_vehicle{
      T_lidar_to_vehicle(obstacle_lidar)};

  std::cout << "Obstacle in Lidar frame: (" << obstacle_lidar.x << ", "
            << obstacle_lidar.y << ", " << obstacle_lidar.z << ")\n";

  std::cout << "Obstacle in Vehicle frame: (" << obstacle_vehicle.x << ", "
            << obstacle_vehicle.y << ", " << obstacle_vehicle.z << ")\n";

  std::cout << "\n";

  // ========================================================================
  // 4. Transform with Rotation
  // ========================================================================
  std::cout << "--- Transform with Rotation ---\n";

  // Radar mounted at 30 degrees to the right
  auto T_radar_to_vehicle =
      rotation_z_transform<RadarFrame, VehicleFrame>(0.523_rad);  // ~30 deg

  const Point3D<RadarFrame> target_radar{10.0_m, 0.0_m, 0.0_m};
  const Point3D<VehicleFrame> target_vehicle{T_radar_to_vehicle(target_radar)};

  std::cout << "Target in Radar frame: (" << target_radar.x << ", "
            << target_radar.y << ", " << target_radar.z << ")\n";

  std::cout << "Target in Vehicle frame: (" << target_vehicle.x << ", "
            << target_vehicle.y << ", " << target_vehicle.z << ")\n";

  std::cout << "\n";

  // ========================================================================
  // 5. Transform Composition
  // ========================================================================
  std::cout << "--- Transform Composition ---\n";

  // Chain transforms: Camera -> Lidar -> Vehicle
  const Transform<CameraFrame, LidarFrame> T_cam_to_lidar{
      translation_transform<CameraFrame, LidarFrame>(0.5_m, 0.2_m, 0.1_m)};

  const Transform<CameraFrame, VehicleFrame> T_cam_to_vehicle{
      T_cam_to_lidar * T_lidar_to_vehicle};

  const Point3D<CameraFrame> feature_camera{5.0_m, 1.0_m, 0.0_m};
  const Point3D<VehicleFrame> feature_vehicle{T_cam_to_vehicle(feature_camera)};
  std::cout << "Feature in Camera frame: (" << feature_camera.x << ", "
            << feature_camera.y << ", " << feature_camera.z << ")\n";

  std::cout << "Feature in Vehicle frame: (" << feature_vehicle.x << ", "
            << feature_vehicle.y << ", " << feature_vehicle.z << ")\n";

  std::cout << "\n";

  // ========================================================================
  // 6. Velocity in Frames
  // ========================================================================
  std::cout << "--- Velocity Vectors ---\n";

  const Velocity3D<VehicleFrame> ego_velocity{25.0_mps, 0.0_mps, 0.0_mps};
  const Velocity3D<MapFrame> obstacle_velocity{15.0_mps, 5.0_mps, 0.0_mps};

  std::cout << "Ego velocity (Vehicle frame): " << ego_velocity.vx << ", "
            << ego_velocity.vy << "\n";

  std::cout << "Obstacle velocity (Map frame): " << obstacle_velocity.vx << ", "
            << obstacle_velocity.vy << "\n";

  std::cout << "Obstacle speed: " << obstacle_velocity.speed() << "\n";

  // ⚠️ Compile-time safety:
  // auto wrong_vel = ego_velocity.vx + obstacle_velocity.vx; // ✓ Same unit
  // But semantic frame mixing is prevented by type design

  std::cout << "\n";

  // ========================================================================
  // 7. Real ADAS Scenario: Obstacle Detection
  // ========================================================================
  std::cout << "--- ADAS Scenario: Multi-Sensor Obstacle Detection ---\n";

  // Lidar detects obstacle
  const Point3D<LidarFrame> lidar_detection{12.0_m, 1.5_m, 0.5_m};
  const Point3D<VehicleFrame> lidar_in_vehicle{
      T_lidar_to_vehicle(lidar_detection)};

  // Radar detects same obstacle
  const Point3D<RadarFrame> radar_detection{11.8_m, 1.6_m, 0.4_m};
  const Point3D<VehicleFrame> radar_in_vehicle{
      T_radar_to_vehicle(radar_detection)};
  std::cout << "Lidar detection (Vehicle): (" << lidar_in_vehicle.x << ", "
            << lidar_in_vehicle.y << ")\n";

  std::cout << "Radar detection (Vehicle): (" << radar_in_vehicle.x << ", "
            << radar_in_vehicle.y << ")\n";

  // Calculate distance between detections (fusion consistency check)
  const auto dx{lidar_in_vehicle.x - radar_in_vehicle.x};
  const auto dy{lidar_in_vehicle.y - radar_in_vehicle.y};
  const auto fusion_error{
      Meters(std::sqrt(dx.value * dx.value + dy.value * dy.value))};

  std::cout << "Fusion consistency error: " << fusion_error << "\n";

  if (fusion_error < 0.5_m) {
    std::cout << "✓ Detections consistent - fusing objects\n";
  } else {
    std::cout << "⚠ Large detection discrepancy - review calibration\n";
  }

  std::cout << "\n=== Phase 1 Demo Complete ===\n";

  return 0;
}
