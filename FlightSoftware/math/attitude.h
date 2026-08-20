#pragma once

namespace math {

/** DCM from MRP s. Row-major. */
void dcm_from_mrp(const float s[3], float C[9]);

/** Gram-Schmidt on DCM rows. Leaves C unchanged if a row is degenerate. */
void dcm_reorthogonalize(float C[9]);

/** Geodesic angle between two DCMs, degrees. */
float dcm_geodesic_angle_deg(const float C_a[9], const float C_b[9]);

}  // namespace math
