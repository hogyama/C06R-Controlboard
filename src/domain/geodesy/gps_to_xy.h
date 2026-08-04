#pragma once

#include <stdint.h>

namespace Domain::Geodesy {

// Converts WGS84 latitude/longitude to a local east/north tangent plane.
// A surveyed reference point and its local XY are immutable after construction.
class GpsToXY {
public:
    GpsToXY(
        int32_t reference_latitude_e7,
        int32_t reference_longitude_e7,
        int32_t reference_x_mm,
        int32_t reference_y_mm);

    bool valid() const;

    bool convert(
        int32_t latitude_e7,
        int32_t longitude_e7,
        int32_t& x_mm,
        int32_t& y_mm) const;

private:
    bool valid_;
    double reference_latitude_deg_;
    double reference_longitude_deg_;
    int32_t reference_x_mm_;
    int32_t reference_y_mm_;
    double metres_per_radian_latitude_;
    double metres_per_radian_longitude_;
};

} // namespace Domain::Geodesy
