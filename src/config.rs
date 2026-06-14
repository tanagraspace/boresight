//! Optional TOML config file that bundles every input, so a pass can be run
//! with a single `--config pass.toml` instead of a long command line. Values in
//! the file take precedence over command-line flags; relative `attitude` and
//! `tle_file` paths resolve against the config file's directory.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::Deserialize;

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FileConfig {
    pub attitude: Option<String>,
    pub time_col: Option<String>,
    /// Quaternion column names, scalar-last [x, y, z, w].
    pub quat_cols: Option<Vec<String>>,
    /// The two TLE element lines.
    pub tle: Option<Vec<String>>,
    pub tle_file: Option<String>,
    pub target_lat: Option<f64>,
    pub target_lon: Option<f64>,
    /// Target as ECEF metres [x, y, z].
    pub target_ecef: Option<Vec<f64>>,
    pub target_name: Option<String>,
    pub reference: Option<String>,
    /// Labeled reference markers: name -> ISO-8601 time.
    pub markers: Option<BTreeMap<String, String>>,
    pub carrier_hz: Option<f64>,
    /// Boresight body axis [x, y, z].
    pub boresight: Option<Vec<f64>>,
    pub invert_quat: Option<bool>,
    /// Windows of interest as [[start, end], ...] seconds from the reference.
    pub windows: Option<Vec<Vec<f64>>>,
    pub playback: Option<f64>,
    pub dt: Option<f64>,

    /// Directory of the config file, used to resolve relative paths. Set on load.
    #[serde(skip)]
    pub base_dir: PathBuf,
}

impl FileConfig {
    pub fn load(path: &str) -> Result<Self> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading config {path}"))?;
        let mut cfg: FileConfig =
            toml::from_str(&text).with_context(|| format!("parsing config {path}"))?;
        cfg.base_dir = Path::new(path)
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_default();
        Ok(cfg)
    }

    /// Resolve a path from the config relative to the config file's directory
    /// (absolute paths are returned unchanged).
    pub fn resolve_path(&self, p: &str) -> String {
        let pb = Path::new(p);
        if pb.is_absolute() {
            p.to_string()
        } else {
            self.base_dir.join(pb).to_string_lossy().into_owned()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_full_toml() {
        let s = r#"
attitude = "a.csv"
quat_cols = ["a", "b", "c", "d"]
tle_file = "s.tle"
target_ecef = [1.0, 2.0, 3.0]
windows = [[0.0, 5.0], [10.0, 15.0]]
playback = 60.0
[markers]
app-start = "2026-06-10T21:21:23Z"
"#;
        let fc: FileConfig = toml::from_str(s).unwrap();
        assert_eq!(fc.attitude.as_deref(), Some("a.csv"));
        assert_eq!(fc.quat_cols.as_ref().unwrap().len(), 4);
        assert_eq!(fc.windows.as_ref().unwrap().len(), 2);
        assert_eq!(fc.playback, Some(60.0));
        assert!(fc.markers.as_ref().unwrap().contains_key("app-start"));
    }

    #[test]
    fn rejects_unknown_field() {
        assert!(toml::from_str::<FileConfig>("bogus = 1").is_err());
    }

    #[test]
    fn resolve_path_relative_and_absolute() {
        let fc = FileConfig {
            base_dir: PathBuf::from("/tmp/pass"),
            ..Default::default()
        };
        assert_eq!(fc.resolve_path("a.csv"), "/tmp/pass/a.csv");
        assert_eq!(fc.resolve_path("/abs/a.csv"), "/abs/a.csv");
    }
}
