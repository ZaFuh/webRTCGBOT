#include "gbot_navigation/coordinates.hpp"
#include <cmath>

namespace Navigation {

Coordinates &Coordinates::operator+=(const Coordinates &rhs) {
  x += rhs.x;
  y += rhs.y;
  return *this;
}

Coordinates &Coordinates::operator-=(const Coordinates &rhs) {
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}

Coordinates &Coordinates::operator*=(int scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}

Coordinates &Coordinates::operator/=(int scalar) {
  x /= scalar;
  y /= scalar;
  return *this;
}

double Coordinates::distance_to(const Coordinates &other) const {
  int dx = x - other.x;
  int dy = y - other.y;
  return std::sqrt(dx * dx + dy * dy);
}

double Coordinates::magnitude() const { return std::sqrt(x * x + y * y); }

bool Coordinates::is_origin() const { return x == 0 && y == 0; }

Coordinates operator+(Coordinates lhs, const Coordinates &rhs) {
  lhs += rhs;
  return lhs;
}

Coordinates operator-(Coordinates lhs, const Coordinates &rhs) {
  lhs -= rhs;
  return lhs;
}

bool operator==(const Coordinates &lhs, const Coordinates &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const Coordinates &lhs, const Coordinates &rhs) {
  return !(lhs == rhs);
}

Coordinates operator*(Coordinates lhs, int scalar) {
  lhs *= scalar;
  return lhs;
}

Coordinates operator*(int scalar, const Coordinates &rhs) {
  return rhs * scalar; // Reuse the previous operator*
}

Coordinates operator/(Coordinates lhs, int scalar) {
  lhs /= scalar;
  return lhs;
}

bool operator<(const Coordinates &lhs, const Coordinates &rhs) {
  // First compare x coordinates, if equal compare y coordinates
  return (lhs.x < rhs.x) || (lhs.x == rhs.x && lhs.y < rhs.y);
}

bool operator>(const Coordinates &lhs, const Coordinates &rhs) {
  return rhs < lhs;
}

bool operator<=(const Coordinates &lhs, const Coordinates &rhs) {
  return !(rhs < lhs);
}

bool operator>=(const Coordinates &lhs, const Coordinates &rhs) {
  return !(lhs < rhs);
}

std::ostream &operator<<(std::ostream &os, const Coordinates &coord) {
  os << "(" << coord.x << ", " << coord.y << ")";
  return os;
}

} // namespace Navigation
