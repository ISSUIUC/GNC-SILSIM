#pragma once

#include "rocket_state.h"

#include <string>

namespace magnetic_model {

/**
 * Compute the local magnetic field in NED-style components:
 *   x -> North, y -> East, z -> Down
 *
 * @param r_km            Geocentric radius in kilometers
 * @param latitude_deg    Latitude in degrees north of the equator
 * @param longitude_deg   Longitude in degrees east of Greenwich
 * @param days_since_2020 Decimal days since January 1, 2020
 * @param coeff_path      Optional path to IGRF13coeffs.csv
 * @return Magnetic field vector in nanoTesla
 */
Vec3 magnet(double r_km,
            double latitude_deg,
            double longitude_deg,
            double days_since_2020,
            const std::string& coeff_path = "../LookUp/IGRF13coeffs.csv");

} // namespace magnetic_model
