//! Application state: the precomputed timeline, derived events, reference-time
//! markers, playback, and view settings.

use anyhow::{bail, Result};
use chrono::{DateTime, Duration, Utc};
use nalgebra::Vector3;

use crate::orbit::Propagator;
use crate::pointing::{
    axis_label, frame_at, window_table, Frame, Geometry, Track, Window, WindowStats,
};

/// Reference instants. `reference` is the timeline origin (t = 0); windows are
/// measured from it. `boresight_on_target` is computed (the
/// minimum-pointing-error instant). `user` holds any caller-supplied labeled
/// markers, so projects can annotate their own events without the tool knowing
/// what they mean.
pub struct Markers {
    pub reference: DateTime<Utc>,
    pub boresight_on_target: DateTime<Utc>,
    pub user: Vec<(String, DateTime<Utc>)>,
}

pub struct App {
    pub target_name: String,
    pub target_ecef_km: Vector3<f64>,
    pub carrier_hz: Option<f64>,
    pub tle_epoch: DateTime<Utc>,
    /// Configured boresight axis in the body frame, and its short label.
    pub boresight_body: Vector3<f64>,
    pub boresight_label: String,

    /// Precomputed frames over the telemetry span, fixed dt.
    pub frames: Vec<Frame>,
    /// Seconds from the reference time, parallel to `frames`.
    pub times_s: Vec<f64>,

    pub windows: Vec<Window>,
    pub window_stats: Vec<WindowStats>,
    pub window_start_idx: Vec<usize>,

    pub markers: Markers,
    pub idx_elev_peak: usize,
    pub idx_min_err: usize,
    pub idx_eclipse_exit: Option<usize>,
    pub user_marker_idx: Vec<usize>,
    /// Largest pointing error over the pass, for stable boresight-scope scaling.
    pub max_pointing_err_deg: f64,

    // Playback.
    pub cur: usize,
    pub playing: bool,
    pub speed: f64,
    accum_s: f64,
    dt_s: f64,

    // View.
    pub map_zoom: f64,
    pub show_labels: bool,
    pub show_table: bool,
    /// Sampling step (seconds) for the whole-pass table.
    pub pass_step_s: f64,
    /// Row scroll offset for the whole-pass table.
    pub pass_scroll: usize,
    pub should_quit: bool,
}

impl App {
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        track: &Track,
        prop: &Propagator,
        geom: &Geometry,
        reference: DateTime<Utc>,
        user_markers: Vec<(String, DateTime<Utc>)>,
        windows: Vec<Window>,
        target_name: String,
        dt_s: f64,
    ) -> Result<Self> {
        // Sample the telemetry span at fixed dt.
        if !(dt_s.is_finite() && dt_s > 0.0) {
            bail!("dt must be a positive, finite number of seconds (got {dt_s})");
        }
        let span_s = (track.end() - track.start()).num_milliseconds() as f64 / 1000.0;
        // Compute the frame count in f64 and range-check it BEFORE the i64 cast:
        // `as i64` saturates, so a tiny dt would otherwise overflow `n` silently.
        let n_f = (span_s / dt_s).floor() + 1.0;
        if !n_f.is_finite() || n_f > 5_000_000.0 {
            bail!("dt of {dt_s:.3e}s over a {span_s:.0}s span would generate {n_f:.3e} frames; use a larger dt");
        }
        let n = n_f as i64;
        let mut frames = Vec::with_capacity(n as usize);
        let mut times_s = Vec::with_capacity(n as usize);
        for k in 0..n {
            let t = track.start() + Duration::milliseconds((k as f64 * dt_s * 1000.0) as i64);
            frames.push(frame_at(t, track, prop, geom)?);
            times_s.push((t - reference).num_milliseconds() as f64 / 1000.0);
        }

        let window_stats = window_table(reference, &windows, track, prop, geom)?;
        let window_start_idx = windows
            .iter()
            .map(|w| nearest_idx(&times_s, w.start_s))
            .collect();

        let idx_elev_peak = argmax(frames.iter().map(|f| f.elevation_deg));
        let idx_min_err = argmin(frames.iter().map(|f| f.pointing_err_deg));
        let idx_eclipse_exit = first_sunlit_exit(frames.iter().map(|f| f.sunlit));

        let user_marker_idx = user_markers
            .iter()
            .map(|(_, t)| {
                nearest_idx(
                    &times_s,
                    (*t - reference).num_milliseconds() as f64 / 1000.0,
                )
            })
            .collect();

        let max_pointing_err_deg = frames
            .iter()
            .map(|f| f.pointing_err_deg)
            .fold(0.0_f64, f64::max);

        let markers = Markers {
            reference,
            boresight_on_target: frames[idx_min_err].t,
            user: user_markers,
        };

        Ok(Self {
            target_name,
            target_ecef_km: geom.target_ecef_km,
            carrier_hz: geom.carrier_hz,
            tle_epoch: prop.epoch(),
            boresight_body: geom.boresight_body,
            boresight_label: axis_label(geom.boresight_body),
            frames,
            times_s,
            windows,
            window_stats,
            window_start_idx,
            markers,
            idx_elev_peak,
            idx_min_err,
            idx_eclipse_exit,
            user_marker_idx,
            max_pointing_err_deg,
            cur: 0,
            playing: true,
            speed: 1.0,
            accum_s: 0.0,
            dt_s,
            map_zoom: 1.0,
            show_labels: true,
            show_table: false,
            pass_step_s: nice_time_step(span_s / 10.0),
            pass_scroll: 0,
            should_quit: false,
        })
    }

    /// Scale the whole-pass table step, clamped to [1 s, pass span]. A smaller
    /// step means more rows. Resets the scroll offset.
    pub fn scale_pass_step(&mut self, factor: f64) {
        let span = self.times_s.last().copied().unwrap_or(1.0)
            - self.times_s.first().copied().unwrap_or(0.0);
        self.pass_step_s = (self.pass_step_s * factor).clamp(1.0, span.max(1.0));
        self.pass_scroll = 0;
    }

    /// Scroll the whole-pass table by `delta` rows (clamped at the top; the
    /// bottom is clamped in the renderer where the visible height is known).
    pub fn scroll_pass(&mut self, delta: isize) {
        self.pass_scroll = (self.pass_scroll as isize + delta).max(0) as usize;
    }

    pub fn current(&self) -> &Frame {
        &self.frames[self.cur]
    }

    pub fn current_time_s(&self) -> f64 {
        self.times_s[self.cur]
    }

    /// Doppler at the current frame, kHz, or `None` if no carrier is configured.
    pub fn doppler_khz(&self) -> Option<f64> {
        self.carrier_hz
            .map(|c| crate::pointing::doppler_hz(self.current().range_rate_m_s, c) / 1000.0)
    }

    /// Which window (1-based) the current frame falls inside, if any.
    pub fn current_window(&self) -> Option<usize> {
        let s = self.current_time_s();
        self.windows
            .iter()
            .position(|w| s >= w.start_s && s <= w.end_s)
            .map(|i| i + 1)
    }

    /// Advance playback by `real_dt_s` of wall time, scaled by `speed`.
    pub fn tick(&mut self, real_dt_s: f64) {
        if !self.playing || self.frames.len() < 2 {
            return;
        }
        self.accum_s += real_dt_s * self.speed;
        while self.accum_s >= self.dt_s {
            self.accum_s -= self.dt_s;
            if self.cur + 1 < self.frames.len() {
                self.cur += 1;
            } else {
                self.playing = false;
                self.accum_s = 0.0;
                break;
            }
        }
    }

    pub fn step(&mut self, delta: i64) {
        let target = self.cur as i64 + delta;
        self.cur = target.clamp(0, self.frames.len() as i64 - 1) as usize;
    }

    pub fn seek_frac(&mut self, frac: f64) {
        let i = (frac.clamp(0.0, 1.0) * (self.frames.len() - 1) as f64).round() as usize;
        self.cur = i;
    }

    /// Jump to the next/prev event index among the marked instants.
    pub fn jump_event(&mut self, forward: bool) {
        let mut marks: Vec<usize> = self.window_start_idx.clone();
        marks.push(self.idx_elev_peak);
        marks.push(self.idx_min_err);
        if let Some(e) = self.idx_eclipse_exit {
            marks.push(e);
        }
        marks.extend(&self.user_marker_idx);
        marks.push(nearest_idx(&self.times_s, 0.0)); // reference (t = 0)
        marks.sort_unstable();
        marks.dedup();
        if forward {
            if let Some(&i) = marks.iter().find(|&&i| i > self.cur) {
                self.cur = i;
            }
        } else if let Some(&i) = marks.iter().rev().find(|&&i| i < self.cur) {
            self.cur = i;
        }
    }
}

/// Snap a desired time step to a "nice" value (1, 2, 5, 10, 15, 30, ... s).
fn nice_time_step(ideal_s: f64) -> f64 {
    const NICE: [f64; 9] = [1.0, 2.0, 5.0, 10.0, 15.0, 30.0, 60.0, 120.0, 300.0];
    let ideal = ideal_s.max(1.0);
    *NICE
        .iter()
        .min_by(|a, b| (**a - ideal).abs().total_cmp(&(**b - ideal).abs()))
        .unwrap()
}

/// Index of the first frame where the spacecraft transitions from shadow into
/// sunlight (the first `false -> true` edge), or `None` if there is no such edge.
fn first_sunlit_exit<I: Iterator<Item = bool>>(flags: I) -> Option<usize> {
    let mut prev = None;
    for (i, lit) in flags.enumerate() {
        if prev == Some(false) && lit {
            return Some(i);
        }
        prev = Some(lit);
    }
    None
}

fn nearest_idx(times_s: &[f64], target: f64) -> usize {
    let mut best = 0;
    let mut bestd = f64::INFINITY;
    for (i, &t) in times_s.iter().enumerate() {
        let d = (t - target).abs();
        if d < bestd {
            bestd = d;
            best = i;
        }
    }
    best
}

fn argmax<I: Iterator<Item = f64>>(it: I) -> usize {
    let mut best = 0;
    let mut bestv = f64::NEG_INFINITY;
    for (i, v) in it.enumerate() {
        if v > bestv {
            bestv = v;
            best = i;
        }
    }
    best
}

fn argmin<I: Iterator<Item = f64>>(it: I) -> usize {
    let mut best = 0;
    let mut bestv = f64::INFINITY;
    for (i, v) in it.enumerate() {
        if v < bestv {
            bestv = v;
            best = i;
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orbit::Propagator;
    use crate::pointing::{AttitudeSample, Convention, Geometry, Track, Window};
    use chrono::{Duration, TimeZone};
    use nalgebra::UnitQuaternion;

    const TLE1: &str = "1 58023U 23155H   26161.48719104  .00004557  00000+0  19207-3 0  9996";
    const TLE2: &str = "2 58023  97.5689 243.0215 0003014  75.7594 284.3977 15.23841600147334";

    fn build_app_dt(dt: f64) -> Result<App> {
        let prop = Propagator::from_tle(TLE1, TLE2).unwrap();
        let t0 = Utc.with_ymd_and_hms(2026, 6, 10, 21, 21, 0).unwrap();
        let samples = (0..6)
            .map(|k| AttitudeSample {
                t: t0 + Duration::seconds(k * 10),
                q: UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.02 * k as f64),
            })
            .collect();
        let track = Track {
            samples,
            convention: Convention::BodyToInertial,
        };
        let geom = Geometry {
            target_ecef_km: Vector3::new(3_845_782.0, 1_114_412.25, 4_948_097.5) / 1000.0,
            boresight_body: Vector3::x(),
            carrier_hz: Some(1_296_000_000.0),
        };
        App::build(
            &track,
            &prop,
            &geom,
            t0,
            vec![],
            vec![Window {
                start_s: 20.0,
                end_s: 40.0,
            }],
            "Legnica".into(),
            dt,
        )
    }

    fn build_app() -> App {
        build_app_dt(1.0).unwrap()
    }

    #[test]
    fn first_sunlit_exit_finds_first_false_to_true_edge() {
        // First false -> true transition is at index 3.
        let s = [true, true, false, true, false, true];
        assert_eq!(first_sunlit_exit(s.into_iter()), Some(3));
        // Never in shadow -> no exit.
        assert_eq!(first_sunlit_exit([true, true, true].into_iter()), None);
        // Always in shadow -> no exit.
        assert_eq!(first_sunlit_exit([false, false].into_iter()), None);
        // Starts in shadow then exits at index 1.
        assert_eq!(first_sunlit_exit([false, true, true].into_iter()), Some(1));
        // Empty.
        assert_eq!(first_sunlit_exit(std::iter::empty()), None);
    }

    #[test]
    fn build_rejects_bad_dt() {
        assert!(build_app_dt(0.0).is_err());
        assert!(build_app_dt(-1.0).is_err());
        assert!(build_app_dt(f64::NAN).is_err());
        assert!(build_app_dt(f64::INFINITY).is_err());
        // A dt so tiny the frame count overflows an i64 must be caught BEFORE the
        // cast, not panic/wrap. 50 s span / 1e-9 = 5e10 frames -> rejected.
        assert!(build_app_dt(1e-9).is_err());
        // A normal dt still builds.
        assert!(build_app_dt(1.0).is_ok());
    }

    #[test]
    fn build_derives_timeline_and_indices() {
        let a = build_app();
        assert_eq!(a.frames.len(), 51); // 50 s span at 1 s + 1
        assert!(a.idx_min_err < a.frames.len());
        assert!(a.idx_elev_peak < a.frames.len());
        assert!(a.max_pointing_err_deg.is_finite() && a.max_pointing_err_deg >= 0.0);
        assert_eq!(a.boresight_label, "+X");
        assert_eq!(a.window_stats.len(), 1);
    }

    #[test]
    fn seek_and_step_clamp() {
        let mut a = build_app();
        a.seek_frac(0.0);
        assert_eq!(a.cur, 0);
        a.seek_frac(1.0);
        assert_eq!(a.cur, a.frames.len() - 1);
        a.cur = 0;
        a.step(3);
        assert_eq!(a.cur, 3);
        a.step(-100);
        assert_eq!(a.cur, 0);
    }

    #[test]
    fn tick_advances_by_speed() {
        let mut a = build_app();
        a.cur = 0;
        a.playing = true;
        a.speed = 1.0;
        a.tick(3.5); // 3.5 s of wall time at 1 s/frame -> 3 frames
        assert_eq!(a.cur, 3);
    }

    #[test]
    fn jump_event_moves_forward() {
        let mut a = build_app();
        a.cur = 0;
        a.jump_event(true);
        assert!(a.cur > 0);
    }

    #[test]
    fn doppler_present_only_with_carrier() {
        let mut a = build_app();
        assert!(a.doppler_khz().is_some());
        a.carrier_hz = None;
        assert!(a.doppler_khz().is_none());
    }

    #[test]
    fn current_window_membership() {
        let mut a = build_app();
        a.cur = 0; // t = 0 s, before the window
        assert_eq!(a.current_window(), None);
        a.cur = 30; // t = 30 s, inside [20, 40]
        assert_eq!(a.current_window(), Some(1));
    }
}
