#ifndef SENSOR_PIPELINE_UNITS_HPP
#define SENSOR_PIPELINE_UNITS_HPP

#include <cmath>
#include <concepts>
#include <ostream>
#include <ratio>

namespace sensor_pipeline {

// ============================================================================
// Core Unit Dimension Tags
// ============================================================================

struct Length {};
struct Time {};
struct Velocity {};
struct Frequency {};
struct Acceleration {};
struct Angle {};

// ============================================================================
// Compile-Time Unit Type
// ============================================================================

template <typename Dimension, typename Ratio = std::ratio<1>>
struct Unit {
  using dimension = Dimension;
  using ratio = Ratio;

  double value;

  constexpr explicit Unit(double v = 0.0) : value(v) {
  }

  // Implicit conversion only for same dimension and ratio
  constexpr double get() const {
    return value;
  }

  constexpr Unit operator-() const {
    return Unit(-value);
  }

  // Arithmetic operations (same unit)
  constexpr Unit operator+(const Unit& other) const {
    return Unit(value + other.value);
  }

  constexpr Unit operator-(const Unit& other) const {
    return Unit(value - other.value);
  }

  constexpr Unit operator*(double scalar) const {
    return Unit(value * scalar);
  }

  constexpr Unit operator/(double scalar) const {
    return Unit(value / scalar);
  }

  // Comparison
  constexpr bool operator<(const Unit& other) const {
    return value < other.value;
  }

  constexpr bool operator>(const Unit& other) const {
    return value > other.value;
  }

  constexpr bool operator<=(const Unit& other) const {
    return value <= other.value;
  }

  constexpr bool operator>=(const Unit& other) const {
    return value >= other.value;
  }

  constexpr bool operator==(const Unit& other) const {
    return std::abs(value - other.value) < 1e-9;
  }
};

// ============================================================================
// Unit Concepts
// ============================================================================

template <typename T>
concept HasLength = std::same_as<typename T::dimension, Length>;

template <typename T>
concept HasTime = std::same_as<typename T::dimension, Time>;

template <typename T>
concept HasVelocity = std::same_as<typename T::dimension, Velocity>;

template <typename T>
concept HasFrequency = std::same_as<typename T::dimension, Frequency>;

template <typename T>
concept HasAngle = std::same_as<typename T::dimension, Angle>;

// ============================================================================
// Specific Unit Types
// ============================================================================

// Length units
using Meters = Unit<Length, std::ratio<1>>;
using Kilometers = Unit<Length, std::kilo>;
using Millimeters = Unit<Length, std::milli>;
using Centimeters = Unit<Length, std::centi>;

// Time units
using Seconds = Unit<Time, std::ratio<1>>;
using Milliseconds = Unit<Time, std::milli>;
using Microseconds = Unit<Time, std::micro>;

// Velocity units
using MetersPerSecond = Unit<Velocity, std::ratio<1>>;
using KilometersPerHour =
    Unit<Velocity, std::ratio<5, 18>>;  // 1 km/h = 5/18 m/s

// Frequency units
using Hertz = Unit<Frequency, std::ratio<1>>;

// Acceleration units
using MetersPerSecondSquared = Unit<Acceleration, std::ratio<1>>;

// Angle units
using Radians = Unit<Angle, std::ratio<1>>;
using Degrees =
    Unit<Angle, std::ratio<314159265, 18000000000>>;  // π/180 approximation

// ============================================================================
// Unit Conversions
// ============================================================================

template <typename To, typename From>
  requires std::same_as<typename To::dimension, typename From::dimension>
constexpr To convert(const From& from) {
  constexpr double conversion_factor =
      static_cast<double>(From::ratio::num) * To::ratio::den /
      (static_cast<double>(From::ratio::den) * To::ratio::num);

  return To(from.value * conversion_factor);
}

// ============================================================================
// User-Defined Literals
// ============================================================================

namespace literals {

// Length
constexpr Meters operator""_m(long double v) {
  return Meters(static_cast<double>(v));
}
constexpr Meters operator""_m(unsigned long long v) {
  return Meters(static_cast<double>(v));
}

constexpr Kilometers operator""_km(long double v) {
  return Kilometers(static_cast<double>(v));
}
constexpr Kilometers operator""_km(unsigned long long v) {
  return Kilometers(static_cast<double>(v));
}

constexpr Millimeters operator""_mm(long double v) {
  return Millimeters(static_cast<double>(v));
}
constexpr Millimeters operator""_mm(unsigned long long v) {
  return Millimeters(static_cast<double>(v));
}

// Time
constexpr Seconds operator""_s(long double v) {
  return Seconds(static_cast<double>(v));
}
constexpr Seconds operator""_s(unsigned long long v) {
  return Seconds(static_cast<double>(v));
}

constexpr Milliseconds operator""_ms(long double v) {
  return Milliseconds(static_cast<double>(v));
}
constexpr Milliseconds operator""_ms(unsigned long long v) {
  return Milliseconds(static_cast<double>(v));
}

// Velocity
constexpr MetersPerSecond operator""_mps(long double v) {
  return MetersPerSecond(static_cast<double>(v));
}
constexpr MetersPerSecond operator""_mps(unsigned long long v) {
  return MetersPerSecond(static_cast<double>(v));
}

constexpr KilometersPerHour operator""_kph(long double v) {
  return KilometersPerHour(static_cast<double>(v));
}
constexpr KilometersPerHour operator""_kph(unsigned long long v) {
  return KilometersPerHour(static_cast<double>(v));
}

// Frequency
constexpr Hertz operator""_Hz(long double v) {
  return Hertz(static_cast<double>(v));
}
constexpr Hertz operator""_Hz(unsigned long long v) {
  return Hertz(static_cast<double>(v));
}

// Angle
constexpr Radians operator""_rad(long double v) {
  return Radians(static_cast<double>(v));
}
constexpr Radians operator""_rad(unsigned long long v) {
  return Radians(static_cast<double>(v));
}

constexpr Degrees operator""_deg(long double v) {
  return Degrees(static_cast<double>(v));
}
constexpr Degrees operator""_deg(unsigned long long v) {
  return Degrees(static_cast<double>(v));
}

}  // namespace literals

// ============================================================================
// Stream Output
// ============================================================================

template <typename Dimension, typename Ratio>
std::ostream& operator<<(std::ostream& os, const Unit<Dimension, Ratio>& unit) {
  os << unit.value;

  if constexpr (std::same_as<Dimension, Length>) {
    if constexpr (std::same_as<Ratio, std::ratio<1>>)
      os << " m";
    else if constexpr (std::same_as<Ratio, std::kilo>)
      os << " km";
    else if constexpr (std::same_as<Ratio, std::milli>)
      os << " mm";
    else
      os << " (length)";
  } else if constexpr (std::same_as<Dimension, Time>) {
    if constexpr (std::same_as<Ratio, std::ratio<1>>)
      os << " s";
    else if constexpr (std::same_as<Ratio, std::milli>)
      os << " ms";
    else
      os << " (time)";
  } else if constexpr (std::same_as<Dimension, Velocity>) {
    os << " m/s";
  } else if constexpr (std::same_as<Dimension, Frequency>) {
    os << " Hz";
  } else if constexpr (std::same_as<Dimension, Angle>) {
    if constexpr (std::same_as<Ratio, std::ratio<1>>)
      os << " rad";
    else
      os << " deg";
  }

  return os;
}

}  // namespace sensor_pipeline

#endif  // SENSOR_PIPELINE_UNITS_HPP
