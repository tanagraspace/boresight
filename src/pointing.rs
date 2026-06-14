//! Attitude telemetry loading, interpolation, and per-instant pointing geometry.

use anyhow::{bail, Context, Result};
use chrono::{DateTime, Utc};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::astro::{
    angle_between_deg, ecef_to_eci_mat, eci_to_ecef_mat, is_sunlit, local_up, sun_unit_eci,
    EARTH_RADIUS_KM,
};
use crate::orbit::Propagator;

pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// One attitude sample: a UTC time and the body-to-inertial quaternion.
#[derive(Clone)]
pub struct AttitudeSample {
    pub t: DateTime<Utc>,
    pub q: UnitQuaternion<f64>,
}

/// Quaternion storage convention.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Convention {
    /// Quaternion rotates a body vector into the inertial frame (default).
    BodyToInertial,
    /// Quaternion rotates an inertial vector into the body frame.
    InertialToBody,
}

/// Names of the CSV columns to read. The quaternion is scalar-last `(x, y, z, w)`.
#[derive(Clone)]
pub struct ColumnSpec {
    pub time: String,
    pub x: String,
    pub y: String,
    pub z: String,
    pub w: String,
}

impl Default for ColumnSpec {
    fn default() -> Self {
        Self {
            time: "time".into(),
            x: "qx".into(),
            y: "qy".into(),
            z: "qz".into(),
            w: "qw".into(),
        }
    }
}

/// Loaded attitude track plus the geometry configuration.
pub struct Track {
    pub samples: Vec<AttitudeSample>,
    pub convention: Convention,
}

impl Track {
    /// Load an attitude CSV from a file path.
    pub fn from_csv(path: &str, cols: &ColumnSpec, convention: Convention) -> Result<Self> {
        let file =
            std::fs::File::open(path).with_context(|| format!("opening attitude CSV {path}"))?;
        Self::from_reader(file, cols, convention)
            .with_context(|| format!("reading attitude CSV {path}"))
    }

    /// Load an attitude track from any CSV reader. `cols` names the time and
    /// quaternion columns; the quaternion is scalar-last `(x, y, z, w)` and time
    /// is ISO-8601. Samples are sorted by time.
    pub fn from_reader<R: std::io::Read>(
        src: R,
        cols: &ColumnSpec,
        convention: Convention,
    ) -> Result<Self> {
        let mut rdr = csv::Reader::from_reader(src);
        let headers = rdr.headers()?.clone();
        let col = |name: &str| {
            headers
                .iter()
                .position(|h| h.trim() == name)
                .ok_or_else(|| anyhow::anyhow!("missing column '{name}'"))
        };
        let (ct, cx, cy, cz, ck) = (
            col(&cols.time)?,
            col(&cols.x)?,
            col(&cols.y)?,
            col(&cols.z)?,
            col(&cols.w)?,
        );
        let mut samples = Vec::new();
        for rec in rdr.records() {
            let rec = rec?;
            let t: DateTime<Utc> = rec[ct]
                .trim()
                .parse()
                .with_context(|| format!("parsing time '{}'", &rec[ct]))?;
            let x: f64 = rec[cx].trim().parse()?;
            let y: f64 = rec[cy].trim().parse()?;
            let z: f64 = rec[cz].trim().parse()?;
            let w: f64 = rec[ck].trim().parse()?;
            // Reject non-finite or zero-norm quaternions; normalizing those would
            // yield NaNs that propagate silently into every pointing computation.
            if ![x, y, z, w].iter().all(|c| c.is_finite())
                || (x * x + y * y + z * z + w * w) < 1e-12
            {
                bail!("non-finite or zero quaternion at time {t}");
            }
            let q = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
            samples.push(AttitudeSample { t, q });
        }
        if samples.len() < 2 {
            bail!("attitude track has fewer than 2 samples");
        }
        samples.sort_by_key(|s| s.t);
        Ok(Self {
            samples,
            convention,
        })
    }

    pub fn start(&self) -> DateTime<Utc> {
        self.samples.first().unwrap().t
    }

    pub fn end(&self) -> DateTime<Utc> {
        self.samples.last().unwrap().t
    }

    /// SLERP-interpolated quaternion at `t`, clamped to the telemetry span.
    pub fn quat_at(&self, t: DateTime<Utc>) -> UnitQuaternion<f64> {
        let s = &self.samples;
        if t <= s[0].t {
            return s[0].q;
        }
        if t >= s[s.len() - 1].t {
            return s[s.len() - 1].q;
        }
        // Find the bracketing pair (small linear scan; tracks are short).
        let mut i = 0;
        while i + 1 < s.len() && s[i + 1].t < t {
            i += 1;
        }
        let (a, b) = (&s[i], &s[i + 1]);
        let span = (b.t - a.t).num_milliseconds() as f64;
        let frac = if span > 0.0 {
            ((t - a.t).num_milliseconds() as f64 / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // try_slerp returns None only for the degenerate (near-antipodal)
        // pair, which valid unit attitude quaternions do not produce; fall back
        // to the bracket start rather than panic in that case.
        a.q.try_slerp(&b.q, frac, 1e-6).unwrap_or(a.q)
    }

    /// Direction of the chosen body axis expressed in the inertial frame.
    fn body_axis_eci(&self, q: UnitQuaternion<f64>, axis_body: Vector3<f64>) -> Vector3<f64> {
        match self.convention {
            Convention::BodyToInertial => q * axis_body,
            Convention::InertialToBody => q.inverse() * axis_body,
        }
    }
}

/// Full geometric state at one instant.
#[derive(Clone)]
pub struct Frame {
    pub t: DateTime<Utc>,
    /// Spacecraft position, ECEF km.
    pub sc_ecef_km: Vector3<f64>,
    /// Pointing error of the boresight to the target, degrees.
    pub pointing_err_deg: f64,
    /// Target elevation above the local horizon, degrees.
    pub elevation_deg: f64,
    /// Target azimuth from local north, degrees (0..360).
    pub azimuth_deg: f64,
    /// Slant range spacecraft-to-target, km.
    pub slant_km: f64,
    /// Range rate (positive = opening), m/s.
    pub range_rate_m_s: f64,
    /// Whether the spacecraft is sunlit.
    pub sunlit: bool,
    /// Body-axis unit vectors expressed in ECEF, for the scene
    /// (index 0 = +X boresight, 1 = +Y, 2 = +Z).
    pub body_axes_ecef: [Vector3<f64>; 3],
}

pub struct Geometry {
    /// Target position, ECEF km.
    pub target_ecef_km: Vector3<f64>,
    /// Boresight axis in the body frame (unit).
    pub boresight_body: Vector3<f64>,
    /// Carrier frequency for Doppler, Hz. `None` if no carrier is configured, in
    /// which case the tool reports range rate instead of Doppler.
    pub carrier_hz: Option<f64>,
}

/// One-way Doppler shift (Hz) for a range rate, given a carrier; positive shift
/// = approaching (closing range). The UI applies this only when a carrier is set.
pub fn doppler_hz(range_rate_m_s: f64, carrier_hz: f64) -> f64 {
    -range_rate_m_s / SPEED_OF_LIGHT_M_S * carrier_hz
}

/// Short label for a body-axis vector: "+X" / "-Y" / "+Z" when it is (near) a
/// principal axis, otherwise its components. Used to label the boresight and the
/// scope's transverse axes for any configured boresight, not just +X.
pub fn axis_label(v: Vector3<f64>) -> String {
    let comps = [(v.x, "X"), (v.y, "Y"), (v.z, "Z")];
    let best = (0..3)
        .max_by(|&a, &b| comps[a].0.abs().total_cmp(&comps[b].0.abs()))
        .unwrap();
    let others_small = (0..3).all(|i| i == best || comps[i].0.abs() < 0.05);
    if comps[best].0.abs() > 0.95 && others_small {
        let (val, name) = comps[best];
        format!("{}{}", if val >= 0.0 { "+" } else { "-" }, name)
    } else {
        format!("[{:.2}, {:.2}, {:.2}]", v.x, v.y, v.z)
    }
}

/// Compute the full geometric state at time `t`.
pub fn frame_at(
    t: DateTime<Utc>,
    track: &Track,
    prop: &Propagator,
    geom: &Geometry,
) -> Result<Frame> {
    let sc = prop.position_ecef_km(t)?;
    let q = track.quat_at(t);

    // Line of sight spacecraft -> target, in ECEF then ECI.
    let los_ecef = (geom.target_ecef_km - sc).normalize();
    let los_eci = ecef_to_eci_mat(t) * los_ecef;
    let bore_eci = track.body_axis_eci(q, geom.boresight_body).normalize();
    let pointing_err_deg = angle_between_deg(bore_eci, los_eci);

    // Topocentric elevation and azimuth at the target.
    let d = sc - geom.target_ecef_km; // target -> spacecraft
    let up = local_up(geom.target_ecef_km);
    let dn = d.normalize();
    let elevation_deg = dn.dot(&up).clamp(-1.0, 1.0).asin().to_degrees();
    let east = Vector3::new(0.0, 0.0, 1.0).cross(&up).normalize();
    let north = up.cross(&east);
    let azimuth_deg = d
        .dot(&east)
        .atan2(d.dot(&north))
        .to_degrees()
        .rem_euclid(360.0);

    let slant_km = (geom.target_ecef_km - sc).norm();

    // Range rate by central difference (1 s), m/s.
    let dt = chrono::Duration::milliseconds(500);
    let r_plus = (geom.target_ecef_km - prop.position_ecef_km(t + dt)?).norm();
    let r_minus = (geom.target_ecef_km - prop.position_ecef_km(t - dt)?).norm();
    let range_rate_m_s = (r_plus - r_minus) * 1000.0; // km/s over 1 s -> m/s

    let sun_ecef = eci_to_ecef_mat(t) * sun_unit_eci(t);
    let sunlit = is_sunlit(sc, sun_ecef, EARTH_RADIUS_KM);

    // Body axes in ECEF for the scene.
    let axis_ecef = |ab: Vector3<f64>| eci_to_ecef_mat(t) * track.body_axis_eci(q, ab);
    let body_axes_ecef = [
        axis_ecef(Vector3::new(1.0, 0.0, 0.0)),
        axis_ecef(Vector3::new(0.0, 1.0, 0.0)),
        axis_ecef(Vector3::new(0.0, 0.0, 1.0)),
    ];

    Ok(Frame {
        t,
        sc_ecef_km: sc,
        pointing_err_deg,
        elevation_deg,
        azimuth_deg,
        slant_km,
        range_rate_m_s,
        sunlit,
        body_axes_ecef,
    })
}

/// A time window of interest, in seconds relative to the reference time.
#[derive(Clone, Copy)]
pub struct Window {
    pub start_s: f64,
    pub end_s: f64,
}

/// Per-window pointing/elevation/range-rate summary. Range rate is carrier-free;
/// the UI converts it to Doppler when a carrier is configured.
pub struct WindowStats {
    pub index: usize,
    pub start_s: f64,
    pub end_s: f64,
    pub angle_start: f64,
    pub angle_end: f64,
    pub angle_min: f64,
    pub angle_mean: f64,
    pub elev_start: f64,
    pub elev_end: f64,
    pub rate_start_m_s: f64,
    pub rate_end_m_s: f64,
    pub sunlit_fraction: f64,
}

/// One sampled instant inside a window.
struct WindowSampleRow {
    angle_deg: f64,
    elev_deg: f64,
    rate_m_s: f64,
    sunlit: bool,
}

/// Aggregate per-window statistics from the sampled rows of a single window.
/// Pure arithmetic, separated from the SGP4 sampling so it can be unit-tested
/// against hand-derived expected values. `rows` must be non-empty.
fn summarize_window(
    index: usize,
    start_s: f64,
    end_s: f64,
    rows: &[WindowSampleRow],
) -> WindowStats {
    let n = rows.len() as f64;
    let angle_min = rows
        .iter()
        .map(|r| r.angle_deg)
        .fold(f64::INFINITY, f64::min);
    let angle_mean = rows.iter().map(|r| r.angle_deg).sum::<f64>() / n;
    let sunlit = rows.iter().filter(|r| r.sunlit).count() as f64 / n;
    WindowStats {
        index,
        start_s,
        end_s,
        angle_start: rows.first().unwrap().angle_deg,
        angle_end: rows.last().unwrap().angle_deg,
        angle_min,
        angle_mean,
        elev_start: rows.first().unwrap().elev_deg,
        elev_end: rows.last().unwrap().elev_deg,
        rate_start_m_s: rows.first().unwrap().rate_m_s,
        rate_end_m_s: rows.last().unwrap().rate_m_s,
        sunlit_fraction: sunlit,
    }
}

/// Compute per-window statistics by sampling each window at 1 s.
pub fn window_table(
    reference: DateTime<Utc>,
    windows: &[Window],
    track: &Track,
    prop: &Propagator,
    geom: &Geometry,
) -> Result<Vec<WindowStats>> {
    let mut out = Vec::new();
    for (i, w) in windows.iter().enumerate() {
        let mut rows = Vec::new();
        let mut s = w.start_s;
        while s <= w.end_s + 1e-9 {
            let t = reference + chrono::Duration::milliseconds((s * 1000.0) as i64);
            let f = frame_at(t, track, prop, geom)?;
            rows.push(WindowSampleRow {
                angle_deg: f.pointing_err_deg,
                elev_deg: f.elevation_deg,
                rate_m_s: f.range_rate_m_s,
                sunlit: f.sunlit,
            });
            s += 1.0;
        }
        out.push(summarize_window(i + 1, w.start_s, w.end_s, &rows));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::astro::{angle_between_deg, ecef_to_eci_mat};
    use crate::orbit::Propagator;
    use chrono::{Duration, TimeZone, Utc};

    const TLE1: &str = "1 58023U 23155H   26161.48719104  .00004557  00000+0  19207-3 0  9996";
    const TLE2: &str = "2 58023  97.5689 243.0215 0003014  75.7594 284.3977 15.23841600147334";

    fn ident_track(n: i64, step_s: i64) -> Track {
        let t0 = Utc.with_ymd_and_hms(2026, 6, 10, 21, 21, 0).unwrap();
        let samples = (0..n)
            .map(|k| AttitudeSample {
                t: t0 + Duration::seconds(k * step_s),
                q: UnitQuaternion::identity(),
            })
            .collect();
        Track {
            samples,
            convention: Convention::BodyToInertial,
        }
    }

    fn legnica_geom() -> Geometry {
        Geometry {
            target_ecef_km: Vector3::new(3_845_782.0, 1_114_412.25, 4_948_097.5) / 1000.0,
            boresight_body: Vector3::new(1.0, 0.0, 0.0),
            carrier_hz: Some(1_296_000_000.0),
        }
    }

    #[test]
    fn axis_label_principal_and_general() {
        assert_eq!(axis_label(Vector3::new(1.0, 0.0, 0.0)), "+X");
        assert_eq!(axis_label(Vector3::new(0.0, -1.0, 0.0)), "-Y");
        assert_eq!(axis_label(Vector3::new(0.0, 0.0, 1.0)), "+Z");
        assert!(axis_label(Vector3::new(0.6, 0.6, 0.0)).starts_with('['));
    }

    #[test]
    fn column_spec_default() {
        let c = ColumnSpec::default();
        assert_eq!(
            (c.time, c.x, c.y, c.z, c.w),
            (
                "time".into(),
                "qx".into(),
                "qy".into(),
                "qz".into(),
                "qw".into(),
            )
        );
    }

    #[test]
    fn from_reader_default_columns_and_sort() {
        // Second row is earlier in time; loader must sort.
        let csv = "time,qx,qy,qz,qw\n\
                   2026-06-10T21:21:10Z,0,0,0.70710678,0.70710678\n\
                   2026-06-10T21:21:00Z,0,0,0,1\n";
        let t = Track::from_reader(
            csv.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial,
        )
        .unwrap();
        assert_eq!(t.samples.len(), 2);
        assert!(t.start() < t.end());
        // First (sorted) sample is the identity quaternion.
        assert!(angle_between_deg(t.samples[0].q * Vector3::x(), Vector3::x()) < 1e-6);
    }

    #[test]
    fn from_reader_custom_columns() {
        let csv = "t,a,b,c,d\n2026-06-10T21:21:00Z,0,0,0,1\n2026-06-10T21:21:10Z,0,0,0,1\n";
        let cols = ColumnSpec {
            time: "t".into(),
            x: "a".into(),
            y: "b".into(),
            z: "c".into(),
            w: "d".into(),
        };
        assert!(Track::from_reader(csv.as_bytes(), &cols, Convention::BodyToInertial).is_ok());
    }

    #[test]
    fn from_reader_errors() {
        // Missing a quaternion column.
        let bad = "time,qx,qy,qz\n2026-06-10T21:21:00Z,0,0,0\n2026-06-10T21:21:10Z,0,0,0\n";
        assert!(Track::from_reader(
            bad.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial
        )
        .is_err());
        // Fewer than two samples.
        let one = "time,qx,qy,qz,qw\n2026-06-10T21:21:00Z,0,0,0,1\n";
        assert!(Track::from_reader(
            one.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial
        )
        .is_err());
        // Non-numeric quaternion cell.
        let badq =
            "time,qx,qy,qz,qw\n2026-06-10T21:21:00Z,foo,0,0,1\n2026-06-10T21:21:10Z,0,0,0,1\n";
        assert!(Track::from_reader(
            badq.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial
        )
        .is_err());
        // Unparseable timestamp.
        let badt = "time,qx,qy,qz,qw\nnot-a-time,0,0,0,1\n2026-06-10T21:21:10Z,0,0,0,1\n";
        assert!(Track::from_reader(
            badt.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial
        )
        .is_err());
        // Non-finite and zero-norm quaternions must be rejected, not silently
        // normalized into NaNs.
        let nan =
            "time,qx,qy,qz,qw\n2026-06-10T21:21:00Z,NaN,0,0,1\n2026-06-10T21:21:10Z,0,0,0,1\n";
        assert!(Track::from_reader(
            nan.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial
        )
        .is_err());
        let zero = "time,qx,qy,qz,qw\n2026-06-10T21:21:00Z,0,0,0,0\n2026-06-10T21:21:10Z,0,0,0,1\n";
        assert!(Track::from_reader(
            zero.as_bytes(),
            &ColumnSpec::default(),
            Convention::BodyToInertial
        )
        .is_err());
    }

    #[test]
    fn quat_at_clamps_and_interpolates() {
        let t0 = Utc.with_ymd_and_hms(2026, 6, 10, 21, 21, 0).unwrap();
        let qa = UnitQuaternion::identity();
        let qb = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
        let track = Track {
            samples: vec![
                AttitudeSample { t: t0, q: qa },
                AttitudeSample {
                    t: t0 + Duration::seconds(10),
                    q: qb,
                },
            ],
            convention: Convention::BodyToInertial,
        };
        // Clamp before/after.
        assert!(track.quat_at(t0 - Duration::seconds(5)).angle_to(&qa) < 1e-9);
        assert!(track.quat_at(t0 + Duration::seconds(50)).angle_to(&qb) < 1e-9);
        // Midpoint equals slerp(0.5).
        let mid = track.quat_at(t0 + Duration::seconds(5));
        assert!(mid.angle_to(&qa.slerp(&qb, 0.5)) < 1e-9);
        // Independent check: the midpoint of identity -> 90 deg about Z is a
        // 45 deg rotation, which maps +X to (cos45, sin45, 0).
        let rx = mid * Vector3::x();
        let r = std::f64::consts::FRAC_1_SQRT_2;
        assert!((rx.x - r).abs() < 1e-9, "x {}", rx.x);
        assert!((rx.y - r).abs() < 1e-9, "y {}", rx.y);
        assert!(rx.z.abs() < 1e-9, "z {}", rx.z);
    }

    #[test]
    fn doppler_sign() {
        assert!(
            doppler_hz(1000.0, 1_296_000_000.0) < 0.0,
            "opening range -> red shift"
        );
        assert!(
            doppler_hz(-1000.0, 1_296_000_000.0) > 0.0,
            "closing range -> blue shift"
        );
    }

    #[test]
    fn doppler_known_value() {
        // f_d = -v/c * f0. For v = -7660 m/s (closing) at 1296 MHz:
        // 7660 * 1.296e9 / 299_792_458 = 33_114.1 Hz.
        let d = doppler_hz(-7660.0, 1_296_000_000.0);
        assert!((d - 33_114.1).abs() < 1.0, "got {d}");
    }

    #[test]
    fn summarize_window_known_values() {
        // Hand-built samples; every asserted output is computed by hand.
        let rows = vec![
            WindowSampleRow {
                angle_deg: 10.0,
                elev_deg: 20.0,
                rate_m_s: -100.0,
                sunlit: true,
            },
            WindowSampleRow {
                angle_deg: 6.0,
                elev_deg: 30.0,
                rate_m_s: -50.0,
                sunlit: true,
            },
            WindowSampleRow {
                angle_deg: 8.0,
                elev_deg: 40.0,
                rate_m_s: 0.0,
                sunlit: false,
            },
            WindowSampleRow {
                angle_deg: 4.0,
                elev_deg: 50.0,
                rate_m_s: 50.0,
                sunlit: false,
            },
        ];
        let s = summarize_window(2, 9.0, 12.0, &rows);
        assert_eq!(s.index, 2);
        assert_eq!((s.start_s, s.end_s), (9.0, 12.0));
        assert_eq!(s.angle_start, 10.0);
        assert_eq!(s.angle_end, 4.0);
        assert_eq!(s.angle_min, 4.0);
        assert_eq!(s.angle_mean, 7.0); // (10 + 6 + 8 + 4) / 4
        assert_eq!(s.elev_start, 20.0);
        assert_eq!(s.elev_end, 50.0);
        assert_eq!(s.rate_start_m_s, -100.0);
        assert_eq!(s.rate_end_m_s, 50.0);
        assert_eq!(s.sunlit_fraction, 0.5); // 2 of 4
    }

    #[test]
    fn summarize_window_single_sample() {
        let rows = vec![WindowSampleRow {
            angle_deg: 3.0,
            elev_deg: 80.0,
            rate_m_s: 12.0,
            sunlit: true,
        }];
        let s = summarize_window(1, 0.0, 0.0, &rows);
        assert_eq!(s.angle_min, 3.0);
        assert_eq!(s.angle_mean, 3.0);
        assert_eq!(s.angle_start, s.angle_end);
        assert_eq!(s.sunlit_fraction, 1.0);
    }

    #[test]
    fn frame_at_invariants_and_convention() {
        let prop = Propagator::from_tle(TLE1, TLE2).unwrap();
        let track = ident_track(4, 10);
        let geom = legnica_geom();
        let t = track.start() + Duration::seconds(15);
        let f = frame_at(t, &track, &prop, &geom).unwrap();

        assert!((0.0..=180.0).contains(&f.pointing_err_deg));
        assert!((-90.0..=90.0).contains(&f.elevation_deg));
        assert!((0.0..360.0).contains(&f.azimuth_deg));
        assert!(f.slant_km > 0.0);
        // Doppler readout sign is consistent with the range rate.
        assert_eq!(
            doppler_hz(f.range_rate_m_s, geom.carrier_hz.unwrap()) > 0.0,
            f.range_rate_m_s < 0.0
        );

        // With an identity attitude, the +X boresight in ECI is ECI +X; the
        // reported error must equal the angle to the line of sight in ECI.
        let los_ecef = (geom.target_ecef_km - f.sc_ecef_km).normalize();
        let los_eci = ecef_to_eci_mat(t) * los_ecef;
        let expected = angle_between_deg(Vector3::x(), los_eci);
        assert!((f.pointing_err_deg - expected).abs() < 1e-6);

        // Inverting the convention with a non-identity attitude changes the error.
        let qb = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7);
        let mut tilted = ident_track(4, 10);
        for s in &mut tilted.samples {
            s.q = qb;
        }
        let e_fwd = frame_at(t, &tilted, &prop, &geom).unwrap().pointing_err_deg;
        tilted.convention = Convention::InertialToBody;
        let e_inv = frame_at(t, &tilted, &prop, &geom).unwrap().pointing_err_deg;
        assert!((e_fwd - e_inv).abs() > 1e-3);
    }

    #[test]
    fn window_table_consistency() {
        let prop = Propagator::from_tle(TLE1, TLE2).unwrap();
        let track = ident_track(6, 10);
        let geom = legnica_geom();
        let reference = track.start();
        let windows = vec![Window {
            start_s: 0.0,
            end_s: 5.0,
        }];
        let stats = window_table(reference, &windows, &track, &prop, &geom).unwrap();
        assert_eq!(stats.len(), 1);
        let s = &stats[0];
        assert_eq!(s.index, 1);
        assert_eq!((s.start_s, s.end_s), (0.0, 5.0));
        assert!(s.angle_min <= s.angle_mean + 1e-9);
        assert!((0.0..=1.0).contains(&s.sunlit_fraction));
    }
}
