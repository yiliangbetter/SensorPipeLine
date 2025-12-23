# ADAS Sensor Pipeline - Compile-Time Type Safety Project

A modern C++20 metaprogramming project for ADAS (Advanced Driver Assistance Systems) that provides **compile-time validated sensor pipelines** with type-safe units and coordinate frames.

## 🎯 Project Goals

This project demonstrates advanced C++ metaprogramming techniques applied to **real-world robotics and ADAS systems**, eliminating entire classes of runtime bugs through compile-time validation.

### Key Features
- ✅ **Type-safe units** - No more mixing meters with seconds
- ✅ **Coordinate frame safety** - Prevents accidental frame mixing
- ✅ **Zero runtime overhead** - All checks at compile time
- ✅ **ADAS-focused** - Built for sensor fusion, perception, and planning

---

## 📋 Project Phases

### ✅ Phase 1: Type-Safe Units & Frames (Current)
- [x] Compile-time unit system (Length, Time, Velocity, Frequency, Angle)
- [x] Unit conversions (m ↔ km, m/s ↔ km/h, etc.)
- [x] Coordinate frame types (Vehicle, Lidar, Camera, Radar, Map, IMU)
- [x] Type-safe transforms between frames
- [x] Transform composition
- [x] Compile-time validation via `static_assert`

### 🔜 Phase 2: Compile-Time Pipeline Graph
- [ ] Sensor message types
- [ ] Pipeline DAG representation
- [ ] Compile-time graph validation
- [ ] Rate and latency checking

### 🔜 Phase 3: FSM for Behavior Planning
- [ ] Type-safe state machines
- [ ] Transition validation
- [ ] Reachability analysis

### 🔜 Phase 4: Safety Rules & Static Assertions
- [ ] Safety-critical constraints
- [ ] ISO 26262 inspired checks
- [ ] Compile-time proof of safety properties

### 🔜 Phase 5: Benchmarks & Performance
- [ ] Runtime comparison vs. unchecked code
- [ ] Zero-overhead validation
- [ ] Real-world ADAS scenarios

---

## 🚀 Getting Started

### Prerequisites
- **C++20 compiler** (GCC 10+, Clang 12+, or MSVC 2019+)
- **CMake 3.20+**

### Building

```bash
# Clone or navigate to project directory
cd SensorPipeLine

# Create build directory
mkdir build && cd build

# Configure
cmake ..

# Build
cmake --build .

# Run demo
./phase1_demo

# Run tests
ctest --verbose
```

### Quick Example

```cpp
#include "units.hpp"
#include "frames.hpp"

using namespace sensor_pipeline::literals;

int main() {
    // ✅ Type-safe units
    auto distance = 150.5_m;
    auto time = 5.0_s;
    
    // ❌ This won't compile - different dimensions!
    // auto wrong = distance + time;
    
    // ✅ Coordinate frames
    Point3D<LidarFrame> lidar_point(10.0_m, 5.0_m, 2.0_m);
    
    // ✅ Transform between frames
    Transform<LidarFrame, VehicleFrame> T_lidar_vehicle;
    Point3D<VehicleFrame> vehicle_point = T_lidar_vehicle(lidar_point);
    
    // ❌ This won't compile - frame mismatch!
    // Point3D<VehicleFrame> wrong = lidar_point;
    
    return 0;
}
```

---

## 🧩 Architecture

### Type-Safe Units (`include/units.hpp`)

```cpp
// Define physical dimensions as types
struct Length {};
struct Time {};
struct Velocity {};

// Unit types with compile-time dimension checking
template<typename Dimension, typename Ratio = std::ratio<1>>
struct Unit { /* ... */ };

// Specific units
using Meters = Unit<Length, std::ratio<1>>;
using Kilometers = Unit<Length, std::kilo>;
using MetersPerSecond = Unit<Velocity, std::ratio<1>>;

// User-defined literals
auto distance = 100.0_m;
auto speed = 30.0_mps;
auto freq = 20_Hz;
```

### Coordinate Frames (`include/frames.hpp`)

```cpp
// Frame tags
struct VehicleFrame {};
struct LidarFrame {};
struct CameraFrame {};
struct RadarFrame {};

// Points are tagged with frames
template<Frame F>
struct Point3D {
    using frame = F;
    Meters x, y, z;
};

// Transforms between frames
template<Frame From, Frame To>
struct Transform {
    Point3D<To> operator()(const Point3D<From>& p) const;
};

// Transform composition (automatic frame chaining)
Transform<A, C> = Transform<A, B> * Transform<B, C>;
```

---

## 🔬 Compile-Time Guarantees

All the following errors are caught **at compile time**:

### ❌ Unit Dimension Mismatch
```cpp
auto distance = 100.0_m;
auto time = 5.0_s;
auto wrong = distance + time;  // ❌ Compile error!
```

### ❌ Frame Mismatch
```cpp
Point3D<LidarFrame> p1;
Point3D<VehicleFrame> p2 = p1;  // ❌ Compile error!
```

### ❌ Invalid Transform
```cpp
Transform<LidarFrame, VehicleFrame> T;
Point3D<CameraFrame> camera_point;
auto result = T(camera_point);  // ❌ Compile error!
```

### ❌ Transform Chain Error
```cpp
Transform<A, B> T1;
Transform<C, D> T2;
auto T3 = T1 * T2;  // ❌ Compile error! B ≠ C
```

---

## 🧪 Testing

### Static Assertion Tests
All type safety is validated at compile time:

```cpp
// Dimension checking
static_assert(std::same_as<Meters::dimension, Length>);
static_assert(!std::same_as<Meters::dimension, Time>);

// Frame type checking
static_assert(!std::same_as<VehicleFrame, LidarFrame>);
static_assert(Frame<VehicleFrame>);

// Transform type checking
static_assert(std::same_as<
    Transform<LidarFrame, VehicleFrame>::from_frame,
    LidarFrame
>);
```

### Runtime Tests
Numerical accuracy validation:

```bash
./compile_time_tests
```

---

## 🎓 C++ Metaprogramming Techniques Used

### Phase 1
- ✅ **Type-level programming** - Dimensions and frames as types
- ✅ **Variadic templates** - Flexible unit composition
- ✅ **C++20 Concepts** - Constrain template parameters
- ✅ **`constexpr` functions** - Compile-time computations
- ✅ **`static_assert`** - Compile-time validation
- ✅ **Type traits** - `is_in_frame`, `SameFrame`
- ✅ **User-defined literals** - Ergonomic syntax (`100_m`, `30_mps`)
- ✅ **SFINAE / Concepts** - Prevent invalid operations

---

## 🚗 Real-World ADAS Applications

This design eliminates bugs commonly seen in:

### Sensor Fusion
- ✅ Prevents coordinate frame confusion (Lidar vs. Camera vs. Vehicle)
- ✅ Ensures unit consistency (meters vs. millimeters)
- ✅ Validates transform chains

### Perception Pipelines
- ✅ Type-safe message passing
- ✅ Frame alignment verification
- ✅ Rate consistency checks (future phases)

### Motion Planning
- ✅ Speed limit enforcement (type-safe velocities)
- ✅ Safe distance calculations
- ✅ Collision prediction with correct units

### Localization
- ✅ Map ↔ Vehicle transform validation
- ✅ GPS coordinate safety
- ✅ IMU integration correctness

---

## 📚 References & Inspiration

- ISO 26262 (Functional Safety for Road Vehicles)
- ROS2 TF2 (Transform Library)
- Modern C++ Design Patterns
- Compile-time dimensional analysis
- Type-safe embedded systems

---

## 🤝 Contributing

This is a personal learning project, but feedback and suggestions are welcome!

### Future Enhancements
- [ ] Support for more sensors (USS, GPS, IMU)
- [ ] Compile-time pipeline optimization
- [ ] Integration with ROS2 message types
- [ ] ASIL-D safety pattern library
- [ ] Performance benchmarking suite

---

## 📜 License

MIT License - feel free to use this for learning and inspiration.

---

## 👤 Author

**Robotics & ADAS Engineer**  
Exploring modern C++ metaprogramming for safety-critical systems.

---

## 🔥 Why This Matters

In ADAS and robotics, **runtime errors cost lives**. By moving validation to compile time:

- 🛡️ **Zero runtime checks** - No performance penalty
- 🔒 **Impossible states** - Can't compile invalid code
- 📊 **Self-documenting** - Types encode domain knowledge
- 🚀 **Faster development** - Catch bugs before testing

**This is C++ metaprogramming with purpose.**

---

*Phase 1 Complete ✅ - Ready for Phase 2: Pipeline Graphs*
