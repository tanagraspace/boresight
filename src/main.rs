mod app;
mod astro;
mod config;
mod orbit;
mod pointing;
mod scene;
mod ui;

use config::FileConfig;

use std::io::{self, Stdout};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use chrono::{DateTime, Utc};
use clap::{ArgMatches, CommandFactory, FromArgMatches, Parser};
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use nalgebra::Vector3;
use ratatui::{backend::CrosstermBackend, Terminal};

use app::App;
use astro::lla_to_ecef;
use orbit::Propagator;
use pointing::{window_table, ColumnSpec, Convention, Geometry, Track, Window};

/// Animated terminal visualizer for spacecraft attitude and target pointing.
#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// TOML config file bundling every input (see examples/config). Values in
    /// the file take precedence over the flags below.
    #[arg(long)]
    config: Option<String>,
    /// Attitude CSV (configure column names with --time-col / --quat-cols).
    #[arg(long)]
    attitude: Option<String>,
    /// Name of the time column in the attitude CSV.
    #[arg(long, default_value = "time")]
    time_col: String,
    /// Quaternion column names, scalar-last "x,y,z,w".
    #[arg(long, default_value = "qx,qy,qz,qw")]
    quat_cols: String,
    /// TLE line 1 (use with --tle2, or use --tle-file instead).
    #[arg(long)]
    tle1: Option<String>,
    /// TLE line 2.
    #[arg(long)]
    tle2: Option<String>,
    /// File containing the TLE: the two element lines, optionally preceded by a
    /// name line (2LE or 3LE). Overrides --tle1 / --tle2.
    #[arg(long)]
    tle_file: Option<String>,
    /// Target latitude, degrees (use with --target-lon).
    #[arg(long, allow_hyphen_values = true)]
    target_lat: Option<f64>,
    /// Target longitude, degrees.
    #[arg(long, allow_hyphen_values = true)]
    target_lon: Option<f64>,
    /// Target as ECEF metres "x,y,z" (overrides lat/lon).
    #[arg(long)]
    target_ecef: Option<String>,
    /// Target display name.
    #[arg(long, default_value = "Target")]
    target_name: String,
    /// Reference time (ISO-8601): the t=0 origin for the chart and window
    /// offsets. Defaults to the attitude start.
    #[arg(long)]
    reference: Option<String>,
    /// Labeled reference marker "label=ISO8601", repeatable.
    #[arg(long = "marker", value_name = "LABEL=TIME")]
    markers: Vec<String>,
    /// Carrier frequency in Hz. When set, the readouts show Doppler; otherwise
    /// they show range rate.
    #[arg(long)]
    carrier_hz: Option<f64>,
    /// Boresight body axis "x,y,z" (default +X).
    #[arg(long, default_value = "1,0,0")]
    boresight: String,
    /// Treat the quaternion as inertial-to-body instead of body-to-inertial.
    #[arg(long, default_value_t = false)]
    invert_quat: bool,
    /// Time windows of interest, drawn on the chart and summarized per-window,
    /// "start:end,..." in seconds relative to the reference time.
    #[arg(long)]
    windows: Option<String>,
    /// Initial playback speed multiplier; 0 starts paused.
    #[arg(long, default_value_t = 1.0)]
    playback: f64,
    /// Timeline step in seconds.
    #[arg(long, default_value_t = 1.0)]
    dt: f64,
    /// Print the per-window table and exit (no TUI).
    #[arg(long, default_value_t = false)]
    dump_table: bool,
    /// Render one static frame as text at "WxH" (e.g. 160x48) and exit.
    #[arg(long)]
    snapshot: Option<String>,
    /// Write the whole-pass timeline to a CSV file and exit.
    #[arg(long, value_name = "PATH")]
    export_csv: Option<String>,
    /// CSV row spacing in seconds (default: every frame, i.e. --dt).
    #[arg(long, value_name = "SECONDS", allow_hyphen_values = true)]
    export_csv_step: Option<f64>,
}

/// Whether a CLI argument was explicitly given on the command line (as opposed
/// to coming from its clap default). Defaulted fields are never `None`, so this
/// is the only reliable way to know the user actually typed the flag.
fn explicit(m: &ArgMatches, id: &str) -> bool {
    matches!(
        m.value_source(id),
        Some(clap::parser::ValueSource::CommandLine)
    )
}

/// Resolve a value with the conventional precedence: an explicitly-passed CLI
/// argument wins; otherwise the config-file value; otherwise the CLI default.
fn pick<T>(cli_explicit: bool, cli_value: T, config_value: Option<T>) -> T {
    if cli_explicit {
        cli_value
    } else {
        config_value.unwrap_or(cli_value)
    }
}

fn parse_vec3(s: &str) -> Result<Vector3<f64>> {
    let parts: Vec<f64> = s
        .split(',')
        .map(|p| p.trim().parse::<f64>())
        .collect::<std::result::Result<_, _>>()
        .with_context(|| format!("parsing vector '{s}'"))?;
    if parts.len() != 3 {
        bail!("expected 3 comma-separated numbers, got '{s}'");
    }
    Ok(Vector3::new(parts[0], parts[1], parts[2]))
}

fn parse_windows(s: &str) -> Result<Vec<Window>> {
    let mut out = Vec::new();
    for tok in s.split(',') {
        let (a, b) = tok
            .split_once(':')
            .ok_or_else(|| anyhow::anyhow!("window '{tok}' is not start:end"))?;
        out.push(Window {
            start_s: a.trim().parse()?,
            end_s: b.trim().parse()?,
        });
    }
    Ok(out)
}

/// The widest a single window may span. `window_table` samples each window at
/// 1 s, so this bounds its allocation the way `App::build` bounds the timeline.
const MAX_WINDOW_SPAN_S: f64 = 1_000_000.0;

/// Reject windows that are non-finite, reversed (end before start), or so wide
/// their per-second sampling would allocate an absurd number of rows.
fn validate_windows(windows: &[Window]) -> Result<()> {
    for w in windows {
        if !(w.start_s.is_finite() && w.end_s.is_finite()) || w.end_s < w.start_s {
            bail!(
                "window must have finite start <= end (got {}:{})",
                w.start_s,
                w.end_s
            );
        }
        if w.end_s - w.start_s > MAX_WINDOW_SPAN_S {
            bail!(
                "window {}:{} spans more than {MAX_WINDOW_SPAN_S:.0} s; narrow it",
                w.start_s,
                w.end_s
            );
        }
    }
    Ok(())
}

/// CSV export row stride: one row every `step_s` seconds over a `dt`-second
/// timeline, or every frame (stride 1) when no step is given. Always >= 1.
fn export_stride(step_s: Option<f64>, dt: f64) -> usize {
    step_s
        .map(|s| (s / dt).round() as usize)
        .unwrap_or(1)
        .max(1)
}

fn parse_columns(quat_cols: &str, time_col: &str) -> Result<ColumnSpec> {
    let parts: Vec<&str> = quat_cols.split(',').map(|s| s.trim()).collect();
    if parts.len() != 4 {
        bail!("--quat-cols expects 4 names 'x,y,z,w', got '{quat_cols}'");
    }
    Ok(ColumnSpec {
        time: time_col.to_string(),
        x: parts[0].to_string(),
        y: parts[1].to_string(),
        z: parts[2].to_string(),
        w: parts[3].to_string(),
    })
}

/// Read the two TLE element lines from a file. Accepts 2LE (two lines) or 3LE
/// (name line then two lines); picks the lines beginning with "1 " and "2 ",
/// falling back to the last two non-empty lines.
fn read_tle_file(path: &str) -> Result<(String, String)> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading TLE file {path}"))?;
    parse_tle_lines(&text).with_context(|| format!("in TLE file {path}"))
}

/// Extract the two TLE element lines from text (2LE, or 3LE with a name line);
/// prefers the lines starting with "1 " and "2 ", else the last two non-empty.
fn parse_tle_lines(text: &str) -> Result<(String, String)> {
    let lines: Vec<&str> = text
        .lines()
        .map(|l| l.trim())
        .filter(|l| !l.is_empty())
        .collect();
    let l1 = lines.iter().find(|l| l.starts_with("1 ")).copied();
    let l2 = lines.iter().find(|l| l.starts_with("2 ")).copied();
    match (l1, l2) {
        (Some(a), Some(b)) => Ok((a.to_string(), b.to_string())),
        _ if lines.len() >= 2 => Ok((
            lines[lines.len() - 2].to_string(),
            lines[lines.len() - 1].to_string(),
        )),
        _ => anyhow::bail!("no two element lines found"),
    }
}

fn parse_markers(specs: &[String]) -> Result<Vec<(String, DateTime<Utc>)>> {
    let mut out = Vec::new();
    for spec in specs {
        let (label, time) = spec
            .split_once('=')
            .ok_or_else(|| anyhow::anyhow!("marker '{spec}' is not label=TIME"))?;
        let t: DateTime<Utc> = time
            .trim()
            .parse()
            .with_context(|| format!("parsing marker time '{time}'"))?;
        out.push((label.trim().to_string(), t));
    }
    Ok(out)
}

/// Resolve the TLE, preferring explicit CLI flags (--tle-file / --tle1+2) over
/// the config file (tle / tle_file).
fn resolve_tle(cli: &Cli, fc: &FileConfig) -> Result<(String, String)> {
    if let Some(path) = &cli.tle_file {
        return read_tle_file(path);
    }
    if let (Some(a), Some(b)) = (&cli.tle1, &cli.tle2) {
        return Ok((a.clone(), b.clone()));
    }
    if let Some(t) = &fc.tle {
        if t.len() >= 2 {
            return Ok((t[0].clone(), t[1].clone()));
        }
        bail!("config tle must list the two element lines");
    }
    if let Some(p) = &fc.tle_file {
        return read_tle_file(&fc.resolve_path(p));
    }
    bail!("provide a TLE (--tle-file, --tle1/--tle2, or config tle/tle_file)")
}

/// Resolve the target position (ECEF km), preferring explicit CLI flags
/// (--target-ecef / --target-lat+lon) over the config file.
fn resolve_target(cli: &Cli, fc: &FileConfig) -> Result<Vector3<f64>> {
    if let Some(ecef) = &cli.target_ecef {
        return Ok(parse_vec3(ecef)? / 1000.0);
    }
    if let (Some(lat), Some(lon)) = (cli.target_lat, cli.target_lon) {
        return Ok(lla_to_ecef(lat, lon, 0.0) / 1000.0);
    }
    if let Some(e) = &fc.target_ecef {
        if e.len() == 3 {
            return Ok(Vector3::new(e[0], e[1], e[2]) / 1000.0);
        }
        bail!("config target_ecef must be [x, y, z]");
    }
    if let (Some(lat), Some(lon)) = (fc.target_lat, fc.target_lon) {
        return Ok(lla_to_ecef(lat, lon, 0.0) / 1000.0);
    }
    bail!("provide a target: lat/lon or ECEF, via flags or config")
}

fn main() -> Result<()> {
    let matches = Cli::command().get_matches();
    let cli = Cli::from_arg_matches(&matches)?;
    let fc = match &cli.config {
        Some(p) => FileConfig::load(p)?,
        None => FileConfig::default(),
    };

    let invert = pick(
        explicit(&matches, "invert_quat"),
        cli.invert_quat,
        fc.invert_quat,
    );
    let convention = if invert {
        Convention::InertialToBody
    } else {
        Convention::BodyToInertial
    };

    let time_col = pick(
        explicit(&matches, "time_col"),
        cli.time_col.clone(),
        fc.time_col.clone(),
    );
    let cols = if explicit(&matches, "quat_cols") {
        parse_columns(&cli.quat_cols, &time_col)?
    } else {
        match &fc.quat_cols {
            Some(q) if q.len() == 4 => ColumnSpec {
                time: time_col,
                x: q[0].clone(),
                y: q[1].clone(),
                z: q[2].clone(),
                w: q[3].clone(),
            },
            Some(_) => bail!("config quat_cols must have 4 names [x, y, z, w]"),
            None => parse_columns(&cli.quat_cols, &time_col)?,
        }
    };

    // An explicit --attitude wins; otherwise the config path (resolved against
    // the config file's directory).
    let attitude = if let Some(a) = &cli.attitude {
        a.clone()
    } else if let Some(a) = &fc.attitude {
        fc.resolve_path(a)
    } else {
        bail!("provide an attitude CSV (--attitude or config `attitude`)");
    };
    let track = Track::from_csv(&attitude, &cols, convention)?;

    let (tle1, tle2) = resolve_tle(&cli, &fc)?;
    let prop = Propagator::from_tle(&tle1, &tle2)?;

    let target_ecef_km = resolve_target(&cli, &fc)?;
    if target_ecef_km.norm() < 1.0 || !target_ecef_km.iter().all(|c| c.is_finite()) {
        bail!("target must be a finite position away from the Earth's center");
    }
    let boresight_body = if explicit(&matches, "boresight") {
        parse_vec3(&cli.boresight)?
    } else {
        match &fc.boresight {
            Some(b) if b.len() == 3 => Vector3::new(b[0], b[1], b[2]),
            Some(_) => bail!("config boresight must be [x, y, z]"),
            None => parse_vec3(&cli.boresight)?,
        }
    }
    .normalize();
    // normalize() of a zero or non-finite vector yields NaNs; reject those.
    if !boresight_body.iter().all(|c| c.is_finite()) {
        bail!("boresight must be a nonzero, finite vector");
    }
    let carrier_hz = cli.carrier_hz.or(fc.carrier_hz);

    let geom = Geometry {
        target_ecef_km,
        boresight_body,
        carrier_hz,
    };

    let reference: DateTime<Utc> = match cli.reference.clone().or_else(|| fc.reference.clone()) {
        Some(s) => s.parse().context("parsing reference time")?,
        None => track.start(),
    };

    // SGP4 accuracy degrades as the propagation time drifts from the TLE epoch;
    // a stale element set yields confidently wrong positions, so warn loudly.
    let tle_age_days = (reference - prop.epoch()).num_seconds().abs() as f64 / 86_400.0;
    if tle_age_days > 14.0 {
        eprintln!(
            "warning: TLE epoch is {tle_age_days:.1} days from the reference time; \
             SGP4 positions may be inaccurate. Use a fresher TLE."
        );
    }

    let mut user_markers = parse_markers(&cli.markers)?;
    if let Some(m) = &fc.markers {
        for (label, time) in m {
            user_markers.push((
                label.clone(),
                time.parse()
                    .with_context(|| format!("parsing marker '{label}' time"))?,
            ));
        }
    }

    // An explicit --windows wins; otherwise the config windows.
    let windows = if let Some(c) = &cli.windows {
        parse_windows(c)?
    } else if let Some(ws) = &fc.windows {
        ws.iter()
            .map(|p| match p.as_slice() {
                [s, e] => Ok(Window {
                    start_s: *s,
                    end_s: *e,
                }),
                _ => bail!("config windows entries must be [start, end]"),
            })
            .collect::<Result<Vec<_>>>()?
    } else {
        Vec::new()
    };
    validate_windows(&windows)?;

    let target_name = pick(
        explicit(&matches, "target_name"),
        cli.target_name.clone(),
        fc.target_name.clone(),
    );
    let dt = pick(explicit(&matches, "dt"), cli.dt, fc.dt);
    let playback = pick(explicit(&matches, "playback"), cli.playback, fc.playback);

    if cli.dump_table {
        if windows.is_empty() {
            bail!("--windows required for --dump-table");
        }
        let table = window_table(reference, &windows, &track, &prop, &geom)?;
        println!(
            "Target {} | reference {} | TLE epoch {} ({:.1} h before)",
            target_name,
            reference.format("%Y-%m-%dT%H:%M:%SZ"),
            prop.epoch().format("%Y-%m-%dT%H:%M:%SZ"),
            (reference - prop.epoch()).num_seconds() as f64 / 3600.0,
        );
        for c in &table {
            println!(
                "win{} [{:+.1},{:+.1}] start {:.1} end {:.1} min {:.1} mean {:.1}",
                c.index, c.start_s, c.end_s, c.angle_start, c.angle_end, c.angle_min, c.angle_mean,
            );
        }
        return Ok(());
    }

    let mut app = App::build(
        &track,
        &prop,
        &geom,
        reference,
        user_markers,
        windows,
        target_name,
        dt,
    )?;
    if playback > 0.0 {
        app.speed = playback;
    } else {
        app.playing = false;
    }

    if let Some(path) = &cli.export_csv {
        if let Some(step) = cli.export_csv_step {
            if !(step.is_finite() && step > 0.0) {
                bail!("--export-csv-step must be a positive number of seconds");
            }
        }
        return export_csv(&app, path, cli.export_csv_step);
    }

    if let Some(spec) = &cli.snapshot {
        return print_snapshot(&app, spec);
    }

    run_tui(app)
}

/// Write the whole-pass timeline to a CSV file, one row every `step_s` seconds
/// (default: every frame, i.e. the timeline step). The Doppler column is filled
/// only when a carrier is configured.
fn export_csv(app: &App, path: &str, step_s: Option<f64>) -> Result<()> {
    use std::io::Write;
    let dt = if app.times_s.len() > 1 {
        (app.times_s[1] - app.times_s[0]).max(1e-6)
    } else {
        1.0
    };
    let stride = export_stride(step_s, dt);
    let mut f = std::fs::File::create(path).with_context(|| format!("creating CSV {path}"))?;
    writeln!(
        f,
        "utc,t_s,pointing_err_deg,elevation_deg,azimuth_deg,slant_km,range_rate_m_s,doppler_khz,sunlit,sub_lat_deg,sub_lon_deg"
    )?;
    let mut written = 0usize;
    for (i, fr) in app.frames.iter().enumerate().step_by(stride) {
        let (lon, lat) = scene::subpoint(fr.sc_ecef_km);
        let dopp = match app.carrier_hz {
            Some(hz) => format!(
                "{:.4}",
                pointing::doppler_hz(fr.range_rate_m_s, hz) / 1000.0
            ),
            None => String::new(),
        };
        writeln!(
            f,
            "{},{:.1},{:.4},{:.4},{:.4},{:.4},{:.4},{},{},{:.5},{:.5}",
            fr.t.format("%Y-%m-%dT%H:%M:%SZ"),
            app.times_s[i],
            fr.pointing_err_deg,
            fr.elevation_deg,
            fr.azimuth_deg,
            fr.slant_km,
            fr.range_rate_m_s,
            dopp,
            fr.sunlit as u8,
            lat,
            lon,
        )?;
        written += 1;
    }
    eprintln!("wrote {written} rows to {path}");
    Ok(())
}

/// Render a single frame to a text buffer and print it (for previews and docs).
fn print_snapshot(app: &App, spec: &str) -> Result<()> {
    use ratatui::backend::TestBackend;
    let (w, h) = spec
        .split_once('x')
        .and_then(|(a, b)| Some((a.trim().parse::<u16>().ok()?, b.trim().parse::<u16>().ok()?)))
        .ok_or_else(|| anyhow::anyhow!("--snapshot expects WxH, e.g. 160x48"))?;
    let mut terminal = Terminal::new(TestBackend::new(w, h))?;
    terminal.draw(|f| ui::draw(f, app))?;
    let buf = terminal.backend().buffer().clone();
    for y in 0..buf.area.height {
        let mut line = String::new();
        for x in 0..buf.area.width {
            line.push_str(buf[(x, y)].symbol());
        }
        println!("{}", line.trim_end());
    }
    Ok(())
}

fn run_tui(mut app: App) -> Result<()> {
    let mut terminal = setup_terminal()?;
    // If the render loop panics, restore the terminal before unwinding so the
    // user is not left in raw mode / the alternate screen, then re-emit the
    // original panic message.
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = disable_raw_mode();
        let _ = execute!(io::stdout(), LeaveAlternateScreen);
        default_hook(info);
    }));
    let res = run_loop(&mut terminal, &mut app);
    restore_terminal(&mut terminal)?;
    res
}

fn setup_terminal() -> Result<Terminal<CrosstermBackend<Stdout>>> {
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    Ok(Terminal::new(CrosstermBackend::new(stdout))?)
}

fn restore_terminal(terminal: &mut Terminal<CrosstermBackend<Stdout>>) -> Result<()> {
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    Ok(())
}

fn run_loop(terminal: &mut Terminal<CrosstermBackend<Stdout>>, app: &mut App) -> Result<()> {
    let tick = Duration::from_millis(33); // ~30 fps
    let mut last = Instant::now();
    loop {
        terminal.draw(|f| ui::draw(f, app))?;

        let timeout = tick.saturating_sub(last.elapsed());
        if event::poll(timeout)? {
            if let Event::Key(k) = event::read()? {
                if k.kind == KeyEventKind::Press {
                    handle_key(app, k.code, k.modifiers);
                }
            }
        }

        let now = Instant::now();
        let dt = (now - last).as_secs_f64();
        last = now;
        app.tick(dt);

        if app.should_quit {
            break;
        }
    }
    Ok(())
}

fn handle_key(app: &mut App, code: KeyCode, mods: KeyModifiers) {
    match code {
        KeyCode::Char('q') | KeyCode::Esc => app.should_quit = true,
        KeyCode::Char('c') if mods.contains(KeyModifiers::CONTROL) => app.should_quit = true,
        KeyCode::Char(' ') => app.playing = !app.playing,
        KeyCode::Left => {
            app.playing = false;
            app.step(-1);
        }
        KeyCode::Right => {
            app.playing = false;
            app.step(1);
        }
        KeyCode::Char(',') => {
            app.playing = false;
            app.jump_event(false);
        }
        KeyCode::Char('.') => {
            app.playing = false;
            app.jump_event(true);
        }
        KeyCode::Char('+') | KeyCode::Char('=') => app.speed = (app.speed * 1.5).min(60.0),
        KeyCode::Char('-') | KeyCode::Char('_') => app.speed = (app.speed / 1.5).max(0.1),
        KeyCode::Char('r') => {
            app.cur = 0;
            app.playing = true;
            app.speed = 1.0;
        }
        KeyCode::Home => {
            app.playing = false;
            app.cur = 0;
        }
        KeyCode::End => {
            app.playing = false;
            app.cur = app.frames.len() - 1;
        }
        KeyCode::Char('z') => app.map_zoom = (app.map_zoom * 1.15).min(8.0),
        KeyCode::Char('Z') => app.map_zoom = (app.map_zoom / 1.15).max(0.3),
        KeyCode::Char('l') => app.show_labels = !app.show_labels,
        KeyCode::Char('t') => app.show_table = !app.show_table,
        KeyCode::Char(']') => app.scale_pass_step(0.5), // more rows (finer step)
        KeyCode::Char('[') => app.scale_pass_step(2.0), // fewer rows (coarser step)
        KeyCode::Up => app.scroll_pass(-1),
        KeyCode::Down => app.scroll_pass(1),
        KeyCode::PageUp => app.scroll_pass(-10),
        KeyCode::PageDown => app.scroll_pass(10),
        KeyCode::Char(c @ '0'..='9') => {
            // Scrub to 0%..90% of the timeline.
            app.playing = false;
            let frac = (c as u8 - b'0') as f64 / 10.0;
            app.seek_frac(frac);
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pick_prefers_explicit_then_config_then_default() {
        // Explicit CLI argument wins, even over a config value.
        assert_eq!(pick(true, 40.0, Some(60.0)), 40.0);
        // No explicit CLI: the config value is used.
        assert_eq!(pick(false, 1.0, Some(60.0)), 60.0);
        // No explicit CLI and no config: the CLI default is used.
        assert_eq!(pick(false, 1.0, None), 1.0);
    }

    #[test]
    fn parse_vec3_ok_and_err() {
        assert_eq!(parse_vec3("1,2,3").unwrap(), Vector3::new(1.0, 2.0, 3.0));
        assert!(parse_vec3("1,2").is_err());
        assert!(parse_vec3("a,b,c").is_err());
    }

    #[test]
    fn parse_windows_ok_and_err() {
        let w = parse_windows("0:5, 10:15").unwrap();
        assert_eq!(w.len(), 2);
        assert_eq!((w[0].start_s, w[0].end_s), (0.0, 5.0));
        assert!(parse_windows("0-5").is_err());
    }

    #[test]
    fn validate_windows_accepts_and_rejects() {
        // Forward and single-instant windows are fine.
        assert!(validate_windows(&[
            Window {
                start_s: 0.0,
                end_s: 5.0
            },
            Window {
                start_s: 7.0,
                end_s: 7.0
            },
        ])
        .is_ok());
        // Reversed window.
        assert!(validate_windows(&[Window {
            start_s: 50.0,
            end_s: 10.0
        }])
        .is_err());
        // Non-finite bound.
        assert!(validate_windows(&[Window {
            start_s: f64::NAN,
            end_s: 5.0
        }])
        .is_err());
        // Absurdly wide window.
        assert!(validate_windows(&[Window {
            start_s: 0.0,
            end_s: MAX_WINDOW_SPAN_S + 1.0
        }])
        .is_err());
    }

    #[test]
    fn export_stride_rounds_and_floors_to_one() {
        // No step -> every frame.
        assert_eq!(export_stride(None, 1.0), 1);
        // step/dt, rounded to nearest frame.
        assert_eq!(export_stride(Some(10.0), 1.0), 10);
        assert_eq!(export_stride(Some(30.0), 1.0), 30);
        assert_eq!(export_stride(Some(2.6), 1.0), 3);
        // Sub-dt step still yields at least every frame, never zero.
        assert_eq!(export_stride(Some(0.4), 1.0), 1);
        // Non-unit dt.
        assert_eq!(export_stride(Some(10.0), 2.0), 5);
    }

    #[test]
    fn parse_columns_ok_and_err() {
        let c = parse_columns("a,b,c,d", "t").unwrap();
        assert_eq!((c.time, c.x, c.w), ("t".into(), "a".into(), "d".into()));
        assert!(parse_columns("a,b,c", "t").is_err());
    }

    #[test]
    fn parse_markers_ok_and_err() {
        let m = parse_markers(&["aos=2026-06-10T21:21:23Z".to_string()]).unwrap();
        assert_eq!(m[0].0, "aos");
        assert!(parse_markers(&["nope".to_string()]).is_err());
    }

    #[test]
    fn parse_tle_lines_2le_3le_and_bad() {
        let (a, b) = parse_tle_lines("1 AAA\n2 BBB\n").unwrap();
        assert_eq!((a.as_str(), b.as_str()), ("1 AAA", "2 BBB"));
        let (_, b3) = parse_tle_lines("OBJECT\n1 AAA\n2 BBB\n").unwrap();
        assert_eq!(b3, "2 BBB");
        assert!(parse_tle_lines("just one line").is_err());
    }
}
