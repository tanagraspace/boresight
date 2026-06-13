//! TLE propagation via SGP4, returning spacecraft position in ECEF kilometres.

use anyhow::{Context, Result};
use chrono::{DateTime, Utc};
use nalgebra::Vector3;

use crate::astro::teme_to_ecef;

pub struct Propagator {
    constants: sgp4::Constants,
    epoch: DateTime<Utc>,
}

impl Propagator {
    /// Build from the two TLE lines. An optional object name is accepted but
    /// not required.
    pub fn from_tle(line1: &str, line2: &str) -> Result<Self> {
        let elements = sgp4::Elements::from_tle(None, line1.as_bytes(), line2.as_bytes())
            .context("parsing TLE")?;
        let epoch = elements.datetime.and_utc();
        let constants = sgp4::Constants::from_elements(&elements)
            .context("building SGP4 constants from elements")?;
        Ok(Self { constants, epoch })
    }

    /// TLE epoch (UTC).
    pub fn epoch(&self) -> DateTime<Utc> {
        self.epoch
    }

    /// Spacecraft position in TEME kilometres at time `t`.
    pub fn position_teme_km(&self, t: DateTime<Utc>) -> Result<Vector3<f64>> {
        let minutes = (t - self.epoch).num_milliseconds() as f64 / 60_000.0;
        let prediction = self
            .constants
            .propagate(sgp4::MinutesSinceEpoch(minutes))
            .map_err(|e| anyhow::anyhow!("SGP4 propagation failed at {t}: {e:?}"))?;
        let p = prediction.position;
        Ok(Vector3::new(p[0], p[1], p[2]))
    }

    /// Spacecraft position in ECEF kilometres at time `t`.
    pub fn position_ecef_km(&self, t: DateTime<Utc>) -> Result<Vector3<f64>> {
        Ok(teme_to_ecef(self.position_teme_km(t)?, t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::{TimeZone, Utc};

    const TLE1: &str = "1 58023U 23155H   26161.48719104  .00004557  00000+0  19207-3 0  9996";
    const TLE2: &str = "2 58023  97.5689 243.0215 0003014  75.7594 284.3977 15.23841600147334";

    #[test]
    fn epoch_parsed() {
        let p = Propagator::from_tle(TLE1, TLE2).unwrap();
        assert_eq!(
            p.epoch().format("%Y-%m-%dT%H:%M").to_string(),
            "2026-06-10T11:41"
        );
    }

    #[test]
    fn position_is_leo_and_frame_consistent() {
        let p = Propagator::from_tle(TLE1, TLE2).unwrap();
        let t = Utc.with_ymd_and_hms(2026, 6, 10, 21, 21, 20).unwrap();
        let ecef = p.position_ecef_km(t).unwrap();
        // OPS-SAT is in a ~500 km SSO; radius ~6900 km.
        assert!(
            (6600.0..7300.0).contains(&ecef.norm()),
            "got {}",
            ecef.norm()
        );
        // ECEF is a pure rotation of TEME, so the magnitude is unchanged.
        let teme = p.position_teme_km(t).unwrap();
        assert!((ecef.norm() - teme.norm()).abs() < 1e-6);
    }

    #[test]
    fn bad_tle_errors() {
        assert!(Propagator::from_tle("not", "a tle").is_err());
    }
}
