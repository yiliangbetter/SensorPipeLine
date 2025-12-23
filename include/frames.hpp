#pragma once

#include "units.hpp"
#include <array>
#include <concepts>
#include <type_traits>

namespace sensor_pipeline {

// ============================================================================
// Frame Tags (Coordinate Reference Frames)
// ============================================================================

struct VehicleFrame {};    // Vehicle body frame (center of rear axle, ISO 8855)
struct LidarFrame {};      // Lidar sensor frame
struct CameraFrame {};     // Camera sensor frame
struct RadarFrame {};      // Radar sensor frame
struct MapFrame {};        // World/Map frame (e.g., UTM)
struct IMUFrame {};        // IMU sensor frame

// ============================================================================
// Compile-Time Frame Concept
// ============================================================================

template<typename T>
concept Frame = std::is_empty_v<T> && (
    std::same_as<T, VehicleFrame> ||
    std::same_as<T, LidarFrame> ||
    std::same_as<T, CameraFrame> ||
    std::same_as<T, RadarFrame> ||
    std::same_as<T, MapFrame> ||
    std::same_as<T, IMUFrame>
);

// ============================================================================
// Point in a Specific Frame
// ============================================================================

template<Frame F>
struct Point3D {
    using frame = F;
    
    Meters x;
    Meters y;
    Meters z;
    
    constexpr Point3D(Meters x_ = Meters(0), Meters y_ = Meters(0), Meters z_ = Meters(0))
        : x(x_), y(y_), z(z_) {}
    
    // Prevent implicit conversion between frames
    Point3D(const Point3D&) = default;
    Point3D& operator=(const Point3D&) = default;
    
    constexpr Meters norm() const {
        return Meters(std::sqrt(x.value * x.value + y.value * y.value + z.value * z.value));
    }
};

// ============================================================================
// Velocity in a Specific Frame
// ============================================================================

template<Frame F>
struct Velocity3D {
    using frame = F;
    
    MetersPerSecond vx;
    MetersPerSecond vy;
    MetersPerSecond vz;
    
    constexpr Velocity3D(MetersPerSecond vx_ = MetersPerSecond(0),
                        MetersPerSecond vy_ = MetersPerSecond(0),
                        MetersPerSecond vz_ = MetersPerSecond(0))
        : vx(vx_), vy(vy_), vz(vz_) {}
    
    constexpr MetersPerSecond speed() const {
        return MetersPerSecond(std::sqrt(vx.value * vx.value + 
                                        vy.value * vy.value + 
                                        vz.value * vz.value));
    }
};

// ============================================================================
// Transform between Frames
// ============================================================================

template<Frame From, Frame To>
struct Transform {
    using from_frame = From;
    using to_frame = To;
    
    // Translation vector (in To frame)
    Point3D<To> translation;
    
    // Rotation matrix (3x3) - row-major
    std::array<std::array<double, 3>, 3> rotation;
    
    constexpr Transform()
        : translation()
        , rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}} // Identity
    {}
    
    constexpr Transform(Point3D<To> trans, std::array<std::array<double, 3>, 3> rot)
        : translation(trans)
        , rotation(rot)
    {}
    
    // Apply transform to a point
    constexpr Point3D<To> operator()(const Point3D<From>& p) const {
        // p_to = R * p_from + t
        Meters x = Meters(rotation[0][0] * p.x.value + 
                         rotation[0][1] * p.y.value + 
                         rotation[0][2] * p.z.value) + translation.x;
        
        Meters y = Meters(rotation[1][0] * p.x.value + 
                         rotation[1][1] * p.y.value + 
                         rotation[1][2] * p.z.value) + translation.y;
        
        Meters z = Meters(rotation[2][0] * p.x.value + 
                         rotation[2][1] * p.y.value + 
                         rotation[2][2] * p.z.value) + translation.z;
        
        return Point3D<To>(x, y, z);
    }
    
    // Apply transform to velocity (rotation only, no translation)
    constexpr Velocity3D<To> operator()(const Velocity3D<From>& v) const {
        MetersPerSecond vx = MetersPerSecond(
            rotation[0][0] * v.vx.value + 
            rotation[0][1] * v.vy.value + 
            rotation[0][2] * v.vz.value
        );
        
        MetersPerSecond vy = MetersPerSecond(
            rotation[1][0] * v.vx.value + 
            rotation[1][1] * v.vy.value + 
            rotation[1][2] * v.vz.value
        );
        
        MetersPerSecond vz = MetersPerSecond(
            rotation[2][0] * v.vx.value + 
            rotation[2][1] * v.vy.value + 
            rotation[2][2] * v.vz.value
        );
        
        return Velocity3D<To>(vx, vy, vz);
    }
};

// ============================================================================
// Transform Composition
// ============================================================================

// Compose transforms: T_AC = T_AB * T_BC
template<Frame A, Frame B, Frame C>
constexpr Transform<A, C> operator*(const Transform<A, B>& t_ab, 
                                   const Transform<B, C>& t_bc) {
    // Rotation: R_AC = R_AB * R_BC
    std::array<std::array<double, 3>, 3> r_ac{};
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r_ac[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                r_ac[i][j] += t_ab.rotation[i][k] * t_bc.rotation[k][j];
            }
        }
    }
    
    // Translation: t_AC = R_AB * t_BC + t_AB
    Point3D<C> t_ac;
    t_ac.x = Meters(
        t_ab.rotation[0][0] * t_bc.translation.x.value +
        t_ab.rotation[0][1] * t_bc.translation.y.value +
        t_ab.rotation[0][2] * t_bc.translation.z.value
    ) + t_ab.translation.x;
    
    t_ac.y = Meters(
        t_ab.rotation[1][0] * t_bc.translation.x.value +
        t_ab.rotation[1][1] * t_bc.translation.y.value +
        t_ab.rotation[1][2] * t_bc.translation.z.value
    ) + t_ab.translation.y;
    
    t_ac.z = Meters(
        t_ab.rotation[2][0] * t_bc.translation.x.value +
        t_ab.rotation[2][1] * t_bc.translation.y.value +
        t_ab.rotation[2][2] * t_bc.translation.z.value
    ) + t_ab.translation.z;
    
    return Transform<A, C>(t_ac, r_ac);
}

// ============================================================================
// Helper Functions for Creating Transforms
// ============================================================================

// Create identity transform
template<Frame From, Frame To>
constexpr Transform<From, To> identity_transform() {
    return Transform<From, To>();
}

// Create translation-only transform
template<Frame From, Frame To>
constexpr Transform<From, To> translation_transform(Meters x, Meters y, Meters z) {
    return Transform<From, To>(
        Point3D<To>(x, y, z),
        {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}
    );
}

// Create rotation around Z-axis (yaw)
template<Frame From, Frame To>
constexpr Transform<From, To> rotation_z_transform(Radians yaw) {
    double c = std::cos(yaw.value);
    double s = std::sin(yaw.value);
    
    return Transform<From, To>(
        Point3D<To>(),
        {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}
    );
}

// ============================================================================
// Compile-Time Frame Validation
// ============================================================================

// Check if two types are in the same frame
template<typename T1, typename T2>
concept SameFrame = requires {
    typename T1::frame;
    typename T2::frame;
} && std::same_as<typename T1::frame, typename T2::frame>;

// Type trait to check frame compatibility at compile time
template<typename T, Frame F>
struct is_in_frame : std::false_type {};

template<Frame F>
struct is_in_frame<Point3D<F>, F> : std::true_type {};

template<Frame F>
struct is_in_frame<Velocity3D<F>, F> : std::true_type {};

template<typename T, Frame F>
inline constexpr bool is_in_frame_v = is_in_frame<T, F>::value;

} // namespace sensor_pipeline
