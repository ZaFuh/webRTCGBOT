#ifndef NAVIGATION_COORDINATES_HPP
#define NAVIGATION_COORDINATES_HPP

#include <cmath>
#include <ostream>

namespace Navigation {
struct Coordinates {
  float x;
  float y;

  // Constructors
  Coordinates() : x(0.0f), y(0.0f) {}
  Coordinates(float x_, float y_) : x(x_), y(y_) {}

  // Operations
  Coordinates &operator+=(const Coordinates &rhs);
  Coordinates &operator-=(const Coordinates &rhs);
  Coordinates &operator*=(int scalar);
  Coordinates &operator/=(int scalar);

  // Utility functions
  double distance_to(const Coordinates &other) const;
  double magnitude() const;
  bool is_origin() const;
};

// Operator overloads
Coordinates operator+(Coordinates lhs, const Coordinates &rhs);
Coordinates operator-(Coordinates lhs, const Coordinates &rhs);
Coordinates operator*(Coordinates lhs, int scalar);
Coordinates operator*(int scalar,
                      const Coordinates &rhs); // allow both 2*coord and coord*2
Coordinates operator/(Coordinates lhs, int scalar);
bool operator==(const Coordinates &lhs, const Coordinates &rhs);
bool operator!=(const Coordinates &lhs, const Coordinates &rhs);
bool operator<(
    const Coordinates &lhs,
    const Coordinates &rhs); // useful for storing in ordered containers
bool operator>(const Coordinates &lhs, const Coordinates &rhs);
bool operator<=(const Coordinates &lhs, const Coordinates &rhs);
bool operator>=(const Coordinates &lhs, const Coordinates &rhs);
std::ostream &operator<<(std::ostream &os, const Coordinates &coord);

} // namespace Navigation

#endif // NAVIGATION_COORDINATES_HPP
