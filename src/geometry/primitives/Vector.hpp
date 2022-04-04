//
// NOTE: This class is pretty much a copy of the Vector3 class of geometrycentral but it uses floats instead of doubles
//       This is useful for faster computations
//

#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

struct Vector {
    // Components
    float x, y, z;

    static Vector zero() { return Vector{0., 0., 0.}; }
    static Vector constant(float c) { return Vector{c, c, c}; }
    static Vector infinity() {
        const float inf = ::std::numeric_limits<float>::infinity();
        return Vector{inf, inf, inf};
    }
    static Vector undefined() {
        const float nan = ::std::numeric_limits<float>::quiet_NaN();
        return Vector{nan, nan, nan};
    }

    // Access-by-index
    float& operator[](int index) { return (&x)[index]; }
    float operator[](int index) const { return (&x)[index]; };

    // Overloaded operators
    Vector operator+(const Vector& v) const;
    Vector operator-(const Vector& v) const;
    Vector operator*(float s) const;
    Vector operator/(float s) const;
    Vector& operator+=(const Vector& v);
    Vector& operator-=(const Vector& v);
    Vector& operator*=(const float& s);
    Vector& operator/=(const float& s);
    bool operator==(const Vector& v) const;
    bool operator!=(const Vector& v) const;

    bool operator<(const Vector &rhs) const;

    bool operator>(const Vector &rhs) const;

    bool operator<=(const Vector &rhs) const;

    bool operator>=(const Vector &rhs) const;

    const Vector operator-() const;

    // Other functions
    Vector rotateAround(Vector axis, float theta) const;
    Vector removeComponent(const Vector& unitDir) const; // removes component in direction D
    std::array<Vector, 2> buildTangentBasis() const;      // build a basis orthogonal to D (need not be unit already)
    Vector normalize() const;
    Vector normalizeCutoff(float mag = 0.) const;
    Vector unit() const;

    float norm() const;
    float norm2() const;

    bool isFinite() const;
    bool isDefined() const;

    std::array<float,3> data() const;
};

// Scalar multiplication
template <typename T>
Vector operator*(const T s, const Vector& v);

// Printing
::std::ostream& operator<<(::std::ostream& output, const Vector& v);
::std::istream& operator>>(::std::istream& intput, Vector& v);

float norm(const Vector& v);
float norm2(const Vector& v);

Vector normalize(const Vector& v);
Vector normalizeCutoff(const Vector& v, float mag = 0.);
Vector unit(const Vector& v);

Vector cross(const Vector& u, const Vector& v);
double angle(const Vector& u, const Vector& v);
float angleInPlane(const Vector& u, const Vector& v, const Vector& normal);
float dot(const Vector& u, const Vector& v);
float sum(const Vector& u);
bool isfinite(const Vector& u); // break camel case rule to match std
bool isDefined(const Vector& u);
Vector componentwiseMin(const Vector& u, const Vector& v);
Vector componentwiseMax(const Vector& u, const Vector& v);




inline Vector Vector::operator+(const Vector& v) const { return Vector{x + v.x, y + v.y, z + v.z}; }

inline Vector Vector::operator-(const Vector& v) const { return Vector{x - v.x, y - v.y, z - v.z}; }

inline Vector Vector::operator*(float s) const { return Vector{x * s, y * s, z * s}; }

inline Vector Vector::operator/(float s) const {
    const float r = 1 / s;
    return Vector{x * r, y * r, z * r};
}

inline const Vector Vector::operator-() const { return Vector{-x, -y, -z}; }

template <typename T>
inline Vector operator*(const T s, const Vector& v) {
    return Vector{s * v.x, s * v.y, s * v.z};
}

inline Vector& Vector::operator+=(const Vector& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

inline Vector& Vector::operator-=(const Vector& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

inline Vector& Vector::operator*=(const float& s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

inline Vector& Vector::operator/=(const float& s) {
    x /= s;
    y /= s;
    z /= s;
    return *this;
}

inline bool Vector::operator==(const Vector& other) const { return x == other.x && y == other.y && z == other.z; }

inline bool Vector::operator!=(const Vector& other) const { return !(*this == other); }

inline float Vector::norm() const { return std::sqrt(x * x + y * y + z * z); }
inline float norm(const Vector& v) { return v.norm(); }

inline float Vector::norm2() const { return x * x + y * y + z * z; }
inline float norm2(const Vector& v) { return v.norm2(); }

inline Vector normalize(const Vector& v) { return v.normalize(); }

inline Vector normalizeCutoff(const Vector& v, float mag) { return v.normalizeCutoff(mag); }

inline Vector unit(const Vector& v) { return normalize(v); }

inline Vector cross(const Vector& u, const Vector& v) {
    float x = u.y * v.z - u.z * v.y;
    float y = u.z * v.x - u.x * v.z;
    float z = u.x * v.y - u.y * v.x;
    return Vector{x, y, z};
}

inline float dot(const Vector& u, const Vector& v) { return u.x * v.x + u.y * v.y + u.z * v.z; }
inline float sum(const Vector& u) { return u.x + u.y + u.z; }

inline double angle(const Vector& u, const Vector& v) {
    return std::acos(std::fmax(-1., std::fmin(1., dot(unit(u), unit(v)))));
}

inline float angleInPlane(const Vector& u, const Vector& v, const Vector& normal) {
    // Put u in plane with the normal
    Vector N = unit(normal);
    Vector uPlane = unit(u - dot(u, N) * N);
    Vector basisY = unit(cross(normal, uPlane));

    float xComp = dot(v, uPlane);
    float yComp = dot(v, basisY);

    return ::std::atan2(yComp, xComp);
}

inline bool Vector::isFinite() const { return ::std::isfinite(x) && ::std::isfinite(y) && ::std::isfinite(z); }
inline bool isfinite(const Vector& v) { return v.isFinite(); }

inline bool Vector::isDefined() const { return (!::std::isnan(x)) && (!::std::isnan(y)) && (!::std::isnan(z)); }
inline bool isDefined(const Vector& v) { return v.isDefined(); }


inline Vector componentwiseMin(const Vector& u, const Vector& v) {
    return Vector{std::fmin(u.x, v.x), std::fmin(u.y, v.y), std::fmin(u.z, v.z)};
}

inline Vector componentwiseMax(const Vector& u, const Vector& v) {
    return Vector{std::fmax(u.x, v.x), std::fmax(u.y, v.y), std::fmax(u.z, v.z)};
}

inline Vector Vector::rotateAround(Vector axis, float theta) const {
    Vector thisV = {x, y, z};
    Vector axisN = axis.normalize();
    Vector parallelComp = axisN * dot(thisV, axisN);
    Vector tangentComp = thisV - parallelComp;

    if (tangentComp.norm2() > 0.0) {
        Vector basisX = tangentComp.normalize();
        Vector basisY = cross(axisN, basisX);

        float tangentMag = tangentComp.norm();

        Vector rotatedV = tangentMag * (std::cos(theta) * basisX + std::sin(theta) * basisY);
        return rotatedV + parallelComp;
    } else {
        return parallelComp;
    }
}

inline Vector Vector::normalize() const {
    float r = 1 / std::sqrt(x * x + y * y + z * z);
    return *this * r;
}

inline Vector Vector::normalizeCutoff(float mag) const {
    float len = std::sqrt(x * x + y * y + z * z);
    if (len <= mag) len = 1.;
    float r = 1 / len;
    return *this * r;
}

inline Vector Vector::unit() const { return normalize(); }

inline Vector Vector::removeComponent(const Vector& unitDir) const { return *this - unitDir * dot(unitDir, *this); }

inline std::array<Vector, 2> Vector::buildTangentBasis() const {
    Vector unitDir = normalize();
    Vector testVec{1., 0., 0.};
    if (std::fabs(dot(testVec, unitDir)) > 0.9) {
        testVec = Vector{0., 1., 0.};
    }

    Vector basisX = cross(testVec, unitDir).normalize();
    Vector basisY = cross(unitDir, basisX).normalize();

    return std::array<Vector, 2>{basisX, basisY};
}


inline std::ostream& operator<<(std::ostream& output, const Vector& v) {
    output << "{" << v.x << ", " << v.y << ", " << v.z << "}";
    return output;
}

inline std::istream& operator>>(std::istream& input, Vector& v) {
    float x, y, z;
    input >> x >> y >> z;
    v = Vector{x, y, z};
    return input;
}

inline std::string to_string(Vector vec) {
    std::ostringstream output;
    output << vec;
    return output.str();
}

inline std::array<float, 3> Vector::data() const {
    std::array<float,3> data{x,y,z};
    return data;
}

inline bool Vector::operator<(const Vector &rhs) const {
    if (x < rhs.x)
        return true;
    if (rhs.x < x)
        return false;
    if (y < rhs.y)
        return true;
    if (rhs.y < y)
        return false;
    return z < rhs.z;
}

inline bool Vector::operator>(const Vector &rhs) const {
    return rhs < *this;
}

inline bool Vector::operator<=(const Vector &rhs) const {
    return !(rhs < *this);
}

inline bool Vector::operator>=(const Vector &rhs) const {
    return !(*this < rhs);
}

#endif //VECTOR_HPP
