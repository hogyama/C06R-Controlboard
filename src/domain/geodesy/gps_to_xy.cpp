#include "gps_to_xy.h"

#include <limits.h>
#include <math.h>

namespace Domain::Geodesy {

namespace {

constexpr double DEG_TO_RAD =
    3.1415926535897932384626433832795 / 180.0;
constexpr double WGS84_A_M = 6378137.0;
constexpr double WGS84_E2 = 6.69437999014e-3;
constexpr int32_t LATITUDE_LIMIT_E7 = 900000000;
constexpr int32_t LONGITUDE_LIMIT_E7 = 1800000000;

bool validGeodeticCoordinate(
    int32_t latitude_e7,
    int32_t longitude_e7)
{
    return latitude_e7 >= -LATITUDE_LIMIT_E7 &&
        latitude_e7 <= LATITUDE_LIMIT_E7 &&
        longitude_e7 >= -LONGITUDE_LIMIT_E7 &&
        longitude_e7 <= LONGITUDE_LIMIT_E7;
}

} // namespace

GpsToXY::GpsToXY(
    int32_t reference_latitude_e7,
    int32_t reference_longitude_e7,
    int32_t reference_x_mm,
    int32_t reference_y_mm)
    : valid_(validGeodeticCoordinate(
          reference_latitude_e7,
          reference_longitude_e7)),
      reference_latitude_deg_(
          static_cast<double>(reference_latitude_e7) * 1.0e-7),
      reference_longitude_deg_(
          static_cast<double>(reference_longitude_e7) * 1.0e-7),
      reference_x_mm_(reference_x_mm),
      reference_y_mm_(reference_y_mm),
      metres_per_radian_latitude_(0.0),
      metres_per_radian_longitude_(0.0)
{
    if (!valid_) return;

    const double latitude_rad =
        reference_latitude_deg_ * DEG_TO_RAD;
    const double sine_latitude = sin(latitude_rad);
    const double w = sqrt(
        1.0 - WGS84_E2 * sine_latitude * sine_latitude);
    if (!isfinite(w) || w <= 0.0) {
        valid_ = false;
        return;
    }

    const double meridian_radius =
        WGS84_A_M * (1.0 - WGS84_E2) / (w * w * w);
    const double prime_vertical_radius = WGS84_A_M / w;
    metres_per_radian_latitude_ = meridian_radius;
    metres_per_radian_longitude_ =
        prime_vertical_radius * cos(latitude_rad);
    valid_ =
        isfinite(metres_per_radian_latitude_) &&
        isfinite(metres_per_radian_longitude_) &&
        metres_per_radian_latitude_ > 0.0 &&
        metres_per_radian_longitude_ > 0.0;
}

bool GpsToXY::valid() const
{
    return valid_;
}

bool GpsToXY::convert(
    int32_t latitude_e7,
    int32_t longitude_e7,
    int32_t& x_mm,
    int32_t& y_mm) const
{
    if (!valid_ ||
        !validGeodeticCoordinate(latitude_e7, longitude_e7)) {
        return false;
    }

    const double latitude_deg =
        static_cast<double>(latitude_e7) * 1.0e-7;
    const double longitude_deg =
        static_cast<double>(longitude_e7) * 1.0e-7;
    const double x =
        (longitude_deg - reference_longitude_deg_) *
        DEG_TO_RAD * metres_per_radian_longitude_ * 1000.0 +
        static_cast<double>(reference_x_mm_);
    const double y =
        (latitude_deg - reference_latitude_deg_) *
        DEG_TO_RAD * metres_per_radian_latitude_ * 1000.0 +
        static_cast<double>(reference_y_mm_);
    if (!isfinite(x) || !isfinite(y) ||
        x < static_cast<double>(INT32_MIN) ||
        x > static_cast<double>(INT32_MAX) ||
        y < static_cast<double>(INT32_MIN) ||
        y > static_cast<double>(INT32_MAX)) {
        return false;
    }

    x_mm = static_cast<int32_t>(lround(x));
    y_mm = static_cast<int32_t>(lround(y));
    return true;
}

} // namespace Domain::Geodesy
