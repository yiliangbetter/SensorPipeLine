// use gtest to do the runtime tests!
#include <cassert>
#include <gtest/gtest.h>
#include <iostream>

#include "frames.hpp"
#include "units.hpp"

// Yiliang
// why the negative number seems to be not working here? try to dig into it later

using namespace sensor_pipeline;
using namespace sensor_pipeline::literals;

TEST(RuntimeTest, UnitConversion) {
  constexpr auto m{1000.0_m};
  auto km = convert<Kilometers>(m);
  EXPECT_NEAR(km.value, 1.0, 1e-6);
}

TEST(RuntimeTest, UnitConversio1) {
  constexpr auto kph{36.0_kph};
  auto mps = convert<MetersPerSecond>(kph);
  EXPECT_NEAR(mps.value, 10.0, 1e-6);
}

TEST(RuntimeTest, FrameTransformTests) {
  auto T = translation_transform<LidarFrame, VehicleFrame>(1.0_m, 2.0_m, 3.0_m);
  const Point3D<LidarFrame> p{5.0_m, 10.0_m, 15.0_m};
  const auto result = T(p);

  EXPECT_NEAR(result.x.value, 6.0, 1e-6);
  EXPECT_NEAR(result.y.value, 12.0, 1e-6);
  EXPECT_NEAR(result.z.value, 18.0, 1e-6);
}

TEST(RuntimeTest, VelocityTests) {
  const Velocity3D<VehicleFrame> v{3.0_mps, 4.0_mps, 0.0_mps};
  const auto speed{v.speed()};
  EXPECT_NEAR(speed.value, 5.0, 1e-6);
}

TEST(RuntimeTest, PointDistances) {
  const Point3D<VehicleFrame> p{3.0_m, 4.0_m, 0.0_m};
  const auto dist{p.norm()};
  EXPECT_NEAR(dist.value, 5.0, 1e-6);
}
