#pragma once

#include <stdint.h>

namespace Domain::Geodesy {

// Converts WGS84 latitude/longitude to a local east/north tangent plane.
// The origin is immutable after construction so every consumer uses the same
// coordinate frame for the lifetime of the converter.
class GpsToXY {
public:
    GpsToXY(int32_t origin_latitude_e7, int32_t origin_longitude_e7);

    bool valid() const;

    bool convert(
        int32_t latitude_e7,
        int32_t longitude_e7,
        int32_t& x_mm,
        int32_t& y_mm) const;

private:
    bool valid_;
    double origin_latitude_deg_;
    double origin_longitude_deg_;
    double metres_per_radian_latitude_;
    double metres_per_radian_longitude_;
};

} // namespace Domain::Geodesy
