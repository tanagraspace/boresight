//! Astrodynamics core: GMST, frame rotations, the Sun vector and a cylindrical
//! shadow test, plus small vector helpers.
//!
//! Conventions:
//!   - GMST: IAU 1982 / Meeus polynomial with full elapsed days (the polynomial
//!     already includes the time-of-day, so no separate term is added).
//!   - TEME -> ECEF: rotation about Z by GMST only (polar motion and nutation
//!     ignored; sub-km at LEO altitudes).
//!   - Quaternion is body-to-inertial, scalar-last (x, y, z, w).
//!   - Boresight defaults to Body +X but is configurable by the caller.

use chrono::{DateTime, TimeZone, Utc};
use nalgebra::{Matrix3, Vector3};

pub const WGS84_A: f64 = 6_378_137.0; // semi-major axis, metres
pub const WGS84_E2: f64 = 6.694_379_990_14e-3; // first eccentricity squared
pub const EARTH_RADIUS_KM: f64 = WGS84_A / 1000.0;

fn j2000() -> DateTime<Utc> {
    Utc.with_ymd_and_hms(2000, 1, 1, 12, 0, 0).unwrap()
}

/// Fractional days since the J2000 epoch.
pub fn days_since_j2000(t: DateTime<Utc>) -> f64 {
    (t - j2000()).num_milliseconds() as f64 / 86_400_000.0
}

/// Geodetic latitude/longitude/altitude (deg, deg, m) to ECEF metres (WGS84).
pub fn lla_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Vector3<f64> {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let n = WGS84_A / (1.0 - WGS84_E2 * lat.sin().powi(2)).sqrt();
    Vector3::new(
        (n + alt_m) * lat.cos() * lon.cos(),
        (n + alt_m) * lat.cos() * lon.sin(),
        (n * (1.0 - WGS84_E2) + alt_m) * lat.sin(),
    )
}

/// Greenwich Mean Sidereal Time in radians.
pub fn gmst_rad(t: DateTime<Utc>) -> f64 {
    let days = days_since_j2000(t);
    let tc = days / 36525.0;
    let gmst_deg = 280.460_618_37 + 360.985_647_366_29 * days + 0.000_387_933 * tc * tc
        - tc * tc * tc / 38_710_000.0;
    gmst_deg.rem_euclid(360.0).to_radians()
}

/// Rotation matrix ECEF -> ECI, about +Z by GMST.
pub fn ecef_to_eci_mat(t: DateTime<Utc>) -> Matrix3<f64> {
    let (s, c) = gmst_rad(t).sin_cos();
    Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
}

/// Rotation matrix ECI -> ECEF (transpose of `ecef_to_eci_mat`).
pub fn eci_to_ecef_mat(t: DateTime<Utc>) -> Matrix3<f64> {
    let (s, c) = gmst_rad(t).sin_cos();
    Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0)
}

/// Rotate TEME (SGP4 output) to ECEF; same GMST-only treatment.
pub fn teme_to_ecef(r_teme: Vector3<f64>, t: DateTime<Utc>) -> Vector3<f64> {
    eci_to_ecef_mat(t) * r_teme
}

/// Low-precision Sun unit vector in ECI, good to ~0.01 deg (Astronomical Almanac).
pub fn sun_unit_eci(t: DateTime<Utc>) -> Vector3<f64> {
    let days = days_since_j2000(t);
    let l = (280.460 + 0.985_647_4 * days)
        .rem_euclid(360.0)
        .to_radians();
    let g = (357.528 + 0.985_600_3 * days)
        .rem_euclid(360.0)
        .to_radians();
    let lam = l + 1.915_f64.to_radians() * g.sin() + 0.020_f64.to_radians() * (2.0 * g).sin();
    let eps = (23.439 - 4e-7 * days).to_radians();
    Vector3::new(lam.cos(), eps.cos() * lam.sin(), eps.sin() * lam.sin())
}

/// Cylindrical-shadow test: eclipsed only when on the anti-Sun side and within
/// one Earth radius of the Sun-Earth line. Inputs in km / unit ECEF.
pub fn is_sunlit(r_sat_km: Vector3<f64>, sun_hat_ecef: Vector3<f64>, radius_km: f64) -> bool {
    let proj = r_sat_km.dot(&sun_hat_ecef);
    let perp = (r_sat_km - sun_hat_ecef * proj).norm();
    !(proj < 0.0 && perp < radius_km)
}

/// Angle between two vectors, in degrees.
pub fn angle_between_deg(a: Vector3<f64>, b: Vector3<f64>) -> f64 {
    let cos = (a.dot(&b) / (a.norm() * b.norm())).clamp(-1.0, 1.0);
    cos.acos().to_degrees()
}

/// Local up at an ECEF point: the geodetic vertical (the WGS84 ellipsoid surface
/// normal), which differs from the geocentric radial by up to ~0.19 deg at
/// mid-latitudes. The ellipsoid normal is parallel to `(x, y, z / (1 - e^2))`.
pub fn local_up(r_ecef: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(r_ecef.x, r_ecef.y, r_ecef.z / (1.0 - WGS84_E2)).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dt(y: i32, mo: u32, d: u32, h: u32, mi: u32, s: u32) -> DateTime<Utc> {
        Utc.with_ymd_and_hms(y, mo, d, h, mi, s).unwrap()
    }

    #[test]
    fn gmst_at_j2000_noon() {
        // GMST at J2000 epoch is ~280.46 deg (the polynomial constant term).
        let g = gmst_rad(dt(2000, 1, 1, 12, 0, 0)).to_degrees();
        assert!((g - 280.460_618_37).abs() < 1e-6, "got {g}");
    }

    #[test]
    fn gmst_advances_at_sidereal_rate() {
        // GMST must advance by the IAU mean sidereal rate of 360.98564736629
        // deg per solar day (i.e. 0.98564736629 deg after subtracting 360).
        // This anchors the rate term independently of the constant term above.
        let g0 = gmst_rad(dt(2000, 1, 1, 12, 0, 0)).to_degrees();
        let g1 = gmst_rad(dt(2000, 1, 2, 12, 0, 0)).to_degrees();
        let advance = (g1 - g0).rem_euclid(360.0);
        assert!((advance - 0.985_647_366_29).abs() < 1e-4, "got {advance}");
    }

    #[test]
    fn rotations_are_inverse() {
        let t = dt(2026, 6, 10, 21, 21, 20);
        let m = ecef_to_eci_mat(t) * eci_to_ecef_mat(t);
        let id = Matrix3::identity();
        assert!((m - id).norm() < 1e-12);
    }

    #[test]
    fn ecef_to_eci_rotates_x_by_gmst() {
        // ecef_to_eci is a rotation about +Z by GMST, so it must map the ECEF
        // x-axis to (cos g, sin g, 0). This pins the rotation axis and sign,
        // which the orthogonality test above cannot.
        let t = dt(2026, 6, 10, 21, 21, 20);
        let g = gmst_rad(t);
        let v = ecef_to_eci_mat(t) * Vector3::x();
        assert!((v.x - g.cos()).abs() < 1e-9, "x {}", v.x);
        assert!((v.y - g.sin()).abs() < 1e-9, "y {}", v.y);
        assert!(v.z.abs() < 1e-9, "z {}", v.z);
    }

    #[test]
    fn local_up_is_geodetic_normal() {
        // At geodetic latitude 45 deg, lon 0, the ellipsoid normal points along
        // (cos45, 0, sin45); the geocentric radial would point elsewhere.
        let p = lla_to_ecef(45.0, 0.0, 0.0);
        let up = local_up(p);
        let r = std::f64::consts::FRAC_1_SQRT_2;
        assert!((up.x - r).abs() < 1e-6, "x {}", up.x);
        assert!(up.y.abs() < 1e-9, "y {}", up.y);
        assert!((up.z - r).abs() < 1e-6, "z {}", up.z);
        // It must differ measurably from the pure geocentric radial.
        assert!((up - p.normalize()).norm() > 1e-4);
    }

    #[test]
    fn lla_roundtrip_magnitude() {
        // A point on the equator at sea level is ~one equatorial radius from centre.
        let r = lla_to_ecef(0.0, 0.0, 0.0);
        assert!((r.norm() - WGS84_A).abs() < 1.0);
        assert!((r.x - WGS84_A).abs() < 1.0);
    }

    #[test]
    fn sun_vector_is_unit() {
        let s = sun_unit_eci(dt(2026, 6, 10, 21, 21, 20));
        assert!((s.norm() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn sun_vector_known_direction_at_j2000() {
        // Independently computed from the low-precision almanac formula at J2000
        // (days = 0): ecliptic longitude ~280.38 deg, obliquity ~23.44 deg.
        let s = sun_unit_eci(dt(2000, 1, 1, 12, 0, 0));
        assert!((s.x - 0.180_10).abs() < 1e-3, "x {}", s.x);
        assert!((s.y - -0.902_48).abs() < 1e-3, "y {}", s.y);
        assert!((s.z - -0.391_27).abs() < 1e-3, "z {}", s.z);
    }

    #[test]
    fn sunlit_when_between_sat_and_sun() {
        let sun = Vector3::new(1.0, 0.0, 0.0);
        // Satellite on the sunward side is lit.
        assert!(is_sunlit(
            Vector3::new(7000.0, 0.0, 0.0),
            sun,
            EARTH_RADIUS_KM
        ));
        // Satellite directly behind Earth, near the axis, is eclipsed.
        assert!(!is_sunlit(
            Vector3::new(-7000.0, 0.0, 0.0),
            sun,
            EARTH_RADIUS_KM
        ));
        // Satellite behind Earth but far off-axis is lit.
        assert!(is_sunlit(
            Vector3::new(-7000.0, 9000.0, 0.0),
            sun,
            EARTH_RADIUS_KM
        ));
    }

    #[test]
    fn angle_between_basic() {
        let a = Vector3::new(1.0, 0.0, 0.0);
        let b = Vector3::new(0.0, 1.0, 0.0);
        assert!((angle_between_deg(a, b) - 90.0).abs() < 1e-9);
        assert!(angle_between_deg(a, a) < 1e-9);
    }
}
