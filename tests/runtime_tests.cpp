// use gtest to do the runtime tests!
#include <cassert>
#include <iostream>

#include "frames.hpp"
#include "units.hpp"

// Yiliang
// why the negative number seems to be not working here? try to dig into it later

using namespace sensor_pipeline;
using namespace sensor_pipeline::literals;

void run_runtime_tests() {
  std::cout << "Running runtime validation tests...\n";

  // Unit conversion tests
  {
    constexpr auto m{1000.0_m};
    auto km = convert<Kilometers>(m);
    assert(std::abs(km.value - 1.0) < 1e-6);
    std::cout << "✓ Meter to kilometer conversion\n";
  }

  {
    constexpr auto kph{36.0_kph};
    auto mps = convert<MetersPerSecond>(kph);
    assert(std::abs(mps.value - 10.0) < 1e-6);
    std::cout << "✓ km/h to m/s conversion\n";
  }

  // Frame transform tests
  {
    auto T =
        translation_transform<LidarFrame, VehicleFrame>(1.0_m, 2.0_m, 3.0_m);
    const Point3D<LidarFrame> p({5.0_m, 10.0_m, 15.0_m};
    const auto result = T(p);

    assert(std::abs(result.x.value - 6.0) < 1e-6);
    assert(std::abs(result.y.value - 12.0) < 1e-6);
    assert(std::abs(result.z.value - 18.0) < 1e-6);
    std::cout << "✓ Translation transform\n";
  }

  // Velocity tests
  {
    const Velocity3D<VehicleFrame> v{3.0_mps, 4.0_mps, 0.0_mps};
    const auto speed{v.speed()};
    assert(std::abs(speed.value - 5.0) < 1e-6);
    std::cout << "✓ Velocity magnitude calculation\n";
  }

  // Point distance
  {
    const Point3D<VehicleFrame> p{3.0_m, 4.0_m, 0.0_m};
    const auto dist{p.norm()};
    assert(std::abs(dist.value - 5.0) < 1e-6);
    std::cout << "✓ Point norm calculation\n";
  }

  std::cout << "\n✅ All runtime tests passed!\n";
}

int main() {
  run_runtime_tests();

  return 0;
}
