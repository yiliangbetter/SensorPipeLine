#include <iostream>

#include "frames.hpp"
#include "units.hpp"

// no main needed for compile-time tests!

using namespace sensor_pipeline;
using namespace sensor_pipeline::literals;

// ============================================================================
// Compile-Time Unit Tests (static_assert based)
// ============================================================================

// Test 1: Unit dimensions are enforced
static_assert(std::same_as<Meters::dimension, Length>);
static_assert(std::same_as<Seconds::dimension, Time>);
static_assert(std::same_as<MetersPerSecond::dimension, Velocity>);
static_assert(std::same_as<Hertz::dimension, Frequency>);

// Test 2: Unit conversions preserve values correctly
constexpr auto test_conversion() {
  auto m = Meters(1000.0);
  auto km = convert<Kilometers>(m);
  return km.value == 1.0;
}
static_assert(test_conversion());

// Test 3: Unit arithmetic
constexpr auto test_arithmetic() {
  auto a = Meters(10.0);
  auto b = Meters(5.0);
  auto sum = a + b;
  return sum.value == 15.0;
}
static_assert(test_arithmetic());

// Test 4: Unit comparison
constexpr auto test_comparison() {
  auto a = Meters(10.0);
  auto b = Meters(5.0);
  return a > b && b < a;
}
static_assert(test_comparison());

// ============================================================================
// Compile-Time Frame Tests
// ============================================================================

// Test 5: Frame types are distinct
static_assert(!std::same_as<VehicleFrame, LidarFrame>);
static_assert(!std::same_as<CameraFrame, RadarFrame>);
static_assert(!std::same_as<MapFrame, IMUFrame>);

// Test 6: Points in different frames have different types
static_assert(!std::same_as<Point3D<VehicleFrame>, Point3D<LidarFrame>>);

// Test 7: Frame concept validation
static_assert(Frame<VehicleFrame>);
static_assert(Frame<LidarFrame>);
static_assert(Frame<CameraFrame>);
static_assert(Frame<RadarFrame>);
static_assert(Frame<MapFrame>);
static_assert(Frame<IMUFrame>);

// Test 8: Transform types are correct
static_assert(
    std::same_as<Transform<LidarFrame, VehicleFrame>::from_frame, LidarFrame>);

static_assert(
    std::same_as<Transform<LidarFrame, VehicleFrame>::to_frame, VehicleFrame>);

// Test 9: Identity transform
constexpr auto test_identity_transform() {
  auto T = identity_transform<VehicleFrame, VehicleFrame>();
  const Point3D<VehicleFrame> p{1.0_m, 2.0_m, 3.0_m};
  auto p_transformed = T(p);

  return p_transformed.x.value == 1.0 && p_transformed.y.value == 2.0 &&
         p_transformed.z.value == 3.0;
}
static_assert(test_identity_transform());

// Test 10: Translation transform
constexpr auto test_translation() {
  auto T{translation_transform<LidarFrame, VehicleFrame>(1.0_m, 2.0_m, 3.0_m)};
  const Point3D<LidarFrame> p{0.0_m, 0.0_m, 0.0_m};
  const auto p_transformed{T(p)};

  return p_transformed.x.value == 1.0 && p_transformed.y.value == 2.0 &&
         p_transformed.z.value == 3.0;
}
static_assert(test_translation());

// Test 11: is_in_frame type trait
static_assert(is_in_frame_v<Point3D<VehicleFrame>, VehicleFrame>);
static_assert(!is_in_frame_v<Point3D<VehicleFrame>, LidarFrame>);
static_assert(is_in_frame_v<Velocity3D<MapFrame>, MapFrame>);
static_assert(!is_in_frame_v<Velocity3D<MapFrame>, CameraFrame>);

// Test 12: SameFrame concept
static_assert(SameFrame<Point3D<VehicleFrame>, Velocity3D<VehicleFrame>>);
static_assert(!SameFrame<Point3D<VehicleFrame>, Point3D<LidarFrame>>);

// ============================================================================
// Compile-Time Error Tests (commented out - would fail to compile)
// ============================================================================

void test_compile_errors() {
  // ❌ These would fail at compile time (as intended):

  // 1. Cannot add different dimensions
  // auto wrong1 = Meters(5.0) + Seconds(10.0);

  // 2. Cannot directly assign points from different frames
  // Point3D<VehicleFrame> p1;
  // Point3D<LidarFrame> p2 = p1;  // No implicit conversion!

  // 3. Cannot use wrong transform
  // Transform<LidarFrame, VehicleFrame> T;
  // Point3D<CameraFrame> p;
  // auto result = T(p);  // Type mismatch!

  // 4. Cannot compose incompatible transforms
  // Transform<LidarFrame, VehicleFrame> T1;
  // Transform<RadarFrame, CameraFrame> T2;
  // auto T3 = T1 * T2;  // Frames don't chain!
}

int main() {
  std::cout << "=== Compile-Time Type Safety Tests ===\n\n";

  std::cout << "✅ All static_assert tests passed at compile time!\n\n";
}
