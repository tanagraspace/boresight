//! Terminal layout and widgets.

use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    symbols,
    text::{Line, Span},
    widgets::{
        canvas::Canvas, Axis, Block, Borders, Chart, Dataset, GraphType, Paragraph, Row, Table,
        TableState,
    },
    Frame,
};

use crate::app::App;
use crate::scene;

const RED: Color = Color::Rgb(204, 0, 0);
const BLUE: Color = Color::Rgb(31, 111, 208);
const GOLD: Color = Color::Rgb(214, 178, 32);
const CYAN: Color = Color::Rgb(90, 200, 230);
const DIM: Color = Color::Rgb(90, 90, 100);
const BAND: Color = Color::Rgb(64, 104, 124); // window edge lines (dim cyan)

pub fn draw(f: &mut Frame, app: &App) {
    let root = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // banner
            Constraint::Min(0),    // main
            Constraint::Length(1), // help
        ])
        .split(f.area());

    draw_banner(f, app, root[0]);
    if app.show_table {
        draw_table(f, app, root[1]);
    } else {
        let main = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(56), Constraint::Percentage(44)])
            .split(root[1]);
        let left = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Percentage(42), Constraint::Percentage(58)])
            .split(main[0]);
        draw_groundtrack(f, app, left[0]);
        draw_scope(f, app, left[1]);
        let right = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Min(8),     // pointing-error / elevation chart
                Constraint::Length(7),  // Doppler / range-rate strip
                Constraint::Length(15), // HUD readouts
            ])
            .split(main[1]);
        draw_chart(f, app, right[0]);
        draw_doppler(f, app, right[1]);
        draw_hud(f, app, right[2]);
    }
    draw_help(f, app, root[2]);
}

fn draw_banner(f: &mut Frame, app: &App, area: Rect) {
    let frame = app.current();
    let mut spans = vec![
        Span::styled(
            frame.t.format("%Y-%m-%d %H:%M:%S UTC").to_string(),
            Style::default().add_modifier(Modifier::BOLD),
        ),
        Span::raw("   "),
    ];
    if let Some(c) = app.current_window() {
        spans.push(Span::styled(
            format!("◉ WINDOW {} OF {}", c, app.windows.len()),
            Style::default()
                .fg(Color::Black)
                .bg(CYAN)
                .add_modifier(Modifier::BOLD),
        ));
    } else if !app.windows.is_empty() {
        spans.push(Span::styled("outside windows", Style::default().fg(DIM)));
    }
    spans.push(Span::raw("   "));
    spans.push(if frame.sunlit {
        Span::styled("☀ SUNLIT", Style::default().fg(GOLD))
    } else {
        Span::styled("● SHADOW", Style::default().fg(DIM))
    });
    if app.cur == app.idx_min_err {
        spans.push(Span::raw("   "));
        spans.push(Span::styled(
            "◎ BORESIGHT ON TARGET",
            Style::default()
                .fg(Color::Rgb(60, 230, 90))
                .add_modifier(Modifier::BOLD),
        ));
    }

    let title = format!(" boresight · {} ", app.target_name);
    let p = Paragraph::new(Line::from(spans))
        .block(Block::default().borders(Borders::ALL).title(title));
    f.render_widget(p, area);
}

fn draw_groundtrack(f: &mut Frame, app: &App, area: Rect) {
    let block = Block::default()
        .borders(Borders::ALL)
        .title(" ground track ");
    let inner = block.inner(area);
    let va = if inner.height > 0 {
        inner.width as f64 / (inner.height as f64 * 2.0)
    } else {
        1.0
    };

    let (x, y) = scene::fit_region(scene::ground_data_bbox(app), va, app.map_zoom);
    let view = scene::MapView {
        xmin: x[0],
        xmax: x[1],
        ymin: y[0],
        ymax: y[1],
        cols: inner.width,
        rows: inner.height,
    };
    let canvas = Canvas::default()
        .block(block)
        .x_bounds(x)
        .y_bounds(y)
        .paint(move |ctx| scene::paint_groundtrack(ctx, app, &view));
    f.render_widget(canvas, area);
}

fn draw_scope(f: &mut Frame, app: &App, area: Rect) {
    let err = app.current().pointing_err_deg;
    let err_col = scene::err_color(err, scene::scope_r_max(app.max_pointing_err_deg));
    let title = Line::from(vec![
        Span::raw(format!(" boresight scope · {} ", app.boresight_label)),
        Span::styled(
            format!("{err:.1}°"),
            Style::default().fg(err_col).add_modifier(Modifier::BOLD),
        ),
        Span::raw(" off (deg rings) "),
    ]);
    let block = Block::default().borders(Borders::ALL).title(title);
    let inner = block.inner(area);
    let va = if inner.height > 0 {
        inner.width as f64 / (inner.height as f64 * 2.0)
    } else {
        1.0
    };
    let r = scene::scope_r_max(app.max_pointing_err_deg);
    let canvas = Canvas::default()
        .block(block)
        .x_bounds([-r * va, r * va])
        .y_bounds([-r, r])
        .paint(move |ctx| scene::paint_scope(ctx, app, r));
    f.render_widget(canvas, area);
}

/// Chart x-axis bounds, widened to a unit span when the timeline holds a single
/// distinct time so the axis labels and the playback cursor stay visible.
fn time_bounds(app: &App) -> (f64, f64) {
    let xmin = *app.times_s.first().unwrap_or(&0.0);
    let xmax = *app.times_s.last().unwrap_or(&1.0);
    if (xmax - xmin).abs() < f64::EPSILON {
        (xmin, xmin + 1.0)
    } else {
        (xmin, xmax)
    }
}

fn draw_chart(f: &mut Frame, app: &App, area: Rect) {
    let err: Vec<(f64, f64)> = app
        .times_s
        .iter()
        .zip(&app.frames)
        .map(|(&t, fr)| (t, fr.pointing_err_deg))
        .collect();
    let elev: Vec<(f64, f64)> = app
        .times_s
        .iter()
        .zip(&app.frames)
        .map(|(&t, fr)| (t, fr.elevation_deg))
        .collect();
    let cur_t = app.current_time_s();
    let cursor = vec![(cur_t, 0.0), (cur_t, 90.0)];
    let cursor_col = scene::err_color(
        app.current().pointing_err_deg,
        scene::scope_r_max(app.max_pointing_err_deg),
    );

    let (xmin, xmax) = time_bounds(app);

    // Window edges: a vertical line at each window's start and end, in the same
    // cyan as the WINDOW banner so they read as window markers, not stray lines.
    let mut win_lines: Vec<Vec<(f64, f64)>> = Vec::new();
    for w in &app.windows {
        win_lines.push(vec![(w.start_s, 0.0), (w.start_s, 90.0)]);
        win_lines.push(vec![(w.end_s, 0.0), (w.end_s, 90.0)]);
    }

    let mut datasets: Vec<Dataset> = win_lines
        .iter()
        .map(|b| {
            Dataset::default()
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(BAND))
                .data(b)
        })
        .collect();
    datasets.push(
        Dataset::default()
            .name("pointing err °")
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(RED))
            .data(&err),
    );
    datasets.push(
        Dataset::default()
            .name("elevation °")
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(BLUE))
            .data(&elev),
    );
    datasets.push(
        Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(cursor_col).add_modifier(Modifier::BOLD))
            .data(&cursor),
    );
    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" pointing error & elevation vs time "),
        )
        .x_axis(
            Axis::default()
                .style(Style::default().fg(DIM))
                .bounds([xmin, xmax])
                .labels(vec![
                    Span::raw(format!("{xmin:.0}s")),
                    Span::raw(format!("{:.0}s", (xmin + xmax) / 2.0)),
                    Span::raw(format!("{xmax:.0}s")),
                ]),
        )
        .y_axis(
            Axis::default()
                .style(Style::default().fg(DIM))
                .bounds([0.0, 90.0])
                .labels(vec![Span::raw("0°"), Span::raw("45°"), Span::raw("90°")]),
        );
    f.render_widget(chart, area);
}

/// Width of the HUD label column; long enough for labels like "experiment start".
const HUD_LABEL_W: usize = 17;

/// Doppler-vs-time strip when a carrier is set, otherwise range rate. The signed
/// curve crosses zero at closest approach, the shape an operator tunes against.
fn draw_doppler(f: &mut Frame, app: &App, area: Rect) {
    let (label, unit) = match app.carrier_hz {
        Some(_) => (" doppler ", "kHz"),
        None => (" range rate ", "km/s"),
    };
    let series: Vec<(f64, f64)> = app
        .times_s
        .iter()
        .zip(&app.frames)
        .map(|(&t, fr)| {
            let v = match app.carrier_hz {
                Some(hz) => crate::pointing::doppler_hz(fr.range_rate_m_s, hz) / 1000.0,
                None => fr.range_rate_m_s / 1000.0,
            };
            (t, v)
        })
        .collect();
    let (xmin, xmax) = time_bounds(app);
    let ymax = series.iter().map(|(_, v)| v.abs()).fold(1.0, f64::max) * 1.1;
    let cur_t = app.current_time_s();
    let cursor = vec![(cur_t, -ymax), (cur_t, ymax)];
    let zero = vec![(xmin, 0.0), (xmax, 0.0)];
    let cursor_col = scene::err_color(
        app.current().pointing_err_deg,
        scene::scope_r_max(app.max_pointing_err_deg),
    );
    let datasets = vec![
        Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(DIM))
            .data(&zero),
        Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(GOLD))
            .data(&series),
        Dataset::default()
            .marker(symbols::Marker::Braille)
            .graph_type(GraphType::Line)
            .style(Style::default().fg(cursor_col))
            .data(&cursor),
    ];
    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!("{label}({unit}) ")),
        )
        .x_axis(
            Axis::default()
                .style(Style::default().fg(DIM))
                .bounds([xmin, xmax]),
        )
        .y_axis(
            Axis::default()
                .style(Style::default().fg(DIM))
                .bounds([-ymax, ymax])
                .labels(vec![
                    Span::raw(format!("{:+.0}", -ymax)),
                    Span::raw("0"),
                    Span::raw(format!("{ymax:+.0}")),
                ]),
        );
    f.render_widget(chart, area);
}

fn kv(label: &str, value: String, color: Color) -> Line<'static> {
    Line::from(vec![
        Span::styled(format!("{label:<HUD_LABEL_W$}"), Style::default().fg(DIM)),
        Span::styled(
            value,
            Style::default().fg(color).add_modifier(Modifier::BOLD),
        ),
    ])
}

fn draw_hud(f: &mut Frame, app: &App, area: Rect) {
    let fr = app.current();
    let off = |m: chrono::DateTime<chrono::Utc>| {
        (m - app.markers.reference).num_milliseconds() as f64 / 1000.0
    };
    let err_col = scene::err_color(
        fr.pointing_err_deg,
        scene::scope_r_max(app.max_pointing_err_deg),
    );
    let mut lines = vec![
        kv(
            "pointing error",
            format!("{:.2}°", fr.pointing_err_deg),
            err_col,
        ),
        kv("elevation", format!("{:.1}°", fr.elevation_deg), BLUE),
        kv("azimuth", format!("{:.1}°", fr.azimuth_deg), Color::Gray),
        kv("slant range", format!("{:.0} km", fr.slant_km), Color::Gray),
        match (app.doppler_khz(), app.carrier_hz) {
            (Some(d), Some(c)) => kv("doppler", format!("{d:+.2} kHz @ {:.0} MHz", c / 1e6), GOLD),
            _ => kv(
                "range rate",
                format!("{:+.2} km/s", fr.range_rate_m_s / 1000.0),
                GOLD,
            ),
        },
        kv(
            "sunlit",
            if fr.sunlit {
                "yes".into()
            } else {
                "no (eclipse)".into()
            },
            GOLD,
        ),
        kv(
            "t from ref",
            format!("{:+.1}s", app.current_time_s()),
            Color::White,
        ),
        kv(
            "min pointing",
            format!(
                "{:.2}° @ {:+.1}s",
                app.frames[app.idx_min_err].pointing_err_deg,
                off(app.markers.boresight_on_target)
            ),
            Color::Rgb(60, 230, 90),
        ),
        kv(
            "elev peak",
            format!(
                "{:.1}° @ {:+.1}s",
                app.frames[app.idx_elev_peak].elevation_deg, app.times_s[app.idx_elev_peak]
            ),
            BLUE,
        ),
        kv(
            "TLE epoch",
            format!(
                "{} ({:+.1} h)",
                app.tle_epoch.format("%m-%d %H:%MZ"),
                (app.markers.reference - app.tle_epoch).num_seconds() as f64 / 3600.0
            ),
            Color::Gray,
        ),
        kv(
            "playback",
            format!(
                "{} · {:.1}×",
                if app.playing { "play" } else { "paused" },
                app.speed
            ),
            Color::White,
        ),
    ];
    // Caller-supplied markers, each shown as an offset from the reference.
    for (label, t) in &app.markers.user {
        // Cap below the column width so a space always separates label and value.
        let short: String = label.chars().take(HUD_LABEL_W - 1).collect();
        lines.push(kv(&short, format!("{:+.1}s", off(*t)), Color::Cyan));
    }
    let p = Paragraph::new(lines).block(Block::default().borders(Borders::ALL).title(" readouts "));
    f.render_widget(p, area);
}

/// Label for the rate column and a formatter that renders Doppler (kHz) when a
/// carrier is set, otherwise range rate (km/s).
fn rate_column(app: &App) -> (&'static str, impl Fn(f64) -> String + '_) {
    let label = if app.carrier_hz.is_some() {
        "dopp kHz"
    } else {
        "rate km/s"
    };
    let fmt = move |rate_m_s: f64| match app.carrier_hz {
        Some(hz) => format!("{:+.2}", crate::pointing::doppler_hz(rate_m_s, hz) / 1000.0),
        None => format!("{:+.2}", rate_m_s / 1000.0),
    };
    (label, fmt)
}

fn draw_table(f: &mut Frame, app: &App, area: Rect) {
    // Split: per-window table on top (sized to its rows), whole-pass table below.
    let chunks = if app.windows.is_empty() {
        Layout::default()
            .constraints([Constraint::Percentage(100)])
            .split(area)
    } else {
        let win_h = (app.windows.len() as u16 + 3).min(area.height / 2);
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(win_h), Constraint::Min(0)])
            .split(area)
    };
    if !app.windows.is_empty() {
        draw_window_table(f, app, chunks[0]);
        draw_pass_table(f, app, chunks[1]);
    } else {
        draw_pass_table(f, app, chunks[0]);
    }
}

fn draw_window_table(f: &mut Frame, app: &App, area: Rect) {
    let (rate_col, rate_fmt) = rate_column(app);
    let header = Row::new(vec![
        "win",
        "window (s)",
        "start°",
        "end°",
        "min°",
        "mean°",
        "elev°",
        rate_col,
        "sunlit",
    ])
    .style(Style::default().fg(GOLD).add_modifier(Modifier::BOLD));
    let rows = app.window_stats.iter().map(|c| {
        Row::new(vec![
            c.index.to_string(),
            format!("{:.1}–{:.1}", c.start_s, c.end_s),
            format!("{:.1}", c.angle_start),
            format!("{:.1}", c.angle_end),
            format!("{:.1}", c.angle_min),
            format!("{:.1}", c.angle_mean),
            format!("{:.0}→{:.0}", c.elev_start, c.elev_end),
            format!(
                "{}→{}",
                rate_fmt(c.rate_start_m_s),
                rate_fmt(c.rate_end_m_s)
            ),
            format!("{:.0}%", c.sunlit_fraction * 100.0),
        ])
    });
    let widths = [
        Constraint::Length(4),
        Constraint::Length(13),
        Constraint::Length(7),
        Constraint::Length(7),
        Constraint::Length(7),
        Constraint::Length(7),
        Constraint::Length(9),
        Constraint::Length(15),
        Constraint::Length(7),
    ];
    let table = Table::new(rows, widths).header(header).block(
        Block::default()
            .borders(Borders::ALL)
            .title(" per-window analysis "),
    );
    f.render_widget(table, area);
}

fn draw_pass_table(f: &mut Frame, app: &App, area: Rect) {
    let (rate_col, rate_fmt) = rate_column(app);
    let dt = if app.times_s.len() > 1 {
        (app.times_s[1] - app.times_s[0]).max(1e-6)
    } else {
        1.0
    };
    let stride = ((app.pass_step_s / dt).round() as usize).max(1);
    let header = Row::new(vec![
        "t (s)", "err°", "elev°", "az°", "slant km", rate_col, "sun",
    ])
    .style(Style::default().fg(GOLD).add_modifier(Modifier::BOLD));
    let rows: Vec<Row> = app
        .frames
        .iter()
        .enumerate()
        .step_by(stride)
        .map(|(i, fr)| {
            Row::new(vec![
                format!("{:+.0}", app.times_s[i]),
                format!("{:.1}", fr.pointing_err_deg),
                format!("{:.1}", fr.elevation_deg),
                format!("{:.0}", fr.azimuth_deg),
                format!("{:.0}", fr.slant_km),
                rate_fmt(fr.range_rate_m_s),
                if fr.sunlit { "☀".into() } else { "·".into() },
            ])
        })
        .collect();
    let widths = [
        Constraint::Length(7),
        Constraint::Length(6),
        Constraint::Length(6),
        Constraint::Length(5),
        Constraint::Length(9),
        Constraint::Length(9),
        Constraint::Length(4),
    ];
    let total = rows.len();
    // Clamp the scroll offset to the rows that overflow the visible area.
    let visible = area.height.saturating_sub(3) as usize; // borders + header
    let max_off = total.saturating_sub(visible);
    let off = app.pass_scroll.min(max_off);
    let scroll_hint = if max_off > 0 {
        format!(
            " · rows {}-{}/{} (↑↓)",
            off + 1,
            (off + visible).min(total),
            total
        )
    } else {
        String::new()
    };
    let title = format!(
        " whole pass · step {:.0}s · {total} rows{scroll_hint} ",
        app.pass_step_s,
    );
    let table = Table::new(rows, widths)
        .header(header)
        .block(Block::default().borders(Borders::ALL).title(title));
    let mut state = TableState::default().with_offset(off);
    f.render_stateful_widget(table, area, &mut state);
}

fn draw_help(f: &mut Frame, app: &App, area: Rect) {
    let txt = if app.show_table {
        "↑/↓ scroll · PgUp/PgDn · [ / ] fewer/more rows · t back · q quit"
    } else {
        "space play · ←/→ step · ,/. event · 0-9 scrub · -/+ speed · r reset · z/Z zoom · l labels · t table · q quit"
    };
    let p =
        Paragraph::new(Span::styled(txt, Style::default().fg(DIM))).alignment(Alignment::Center);
    f.render_widget(p, area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::App;
    use crate::astro::lla_to_ecef;
    use crate::orbit::Propagator;
    use crate::pointing::{AttitudeSample, Convention, Geometry, Track, Window};
    use chrono::{Duration, TimeZone, Utc};
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};
    use ratatui::{backend::TestBackend, Terminal};

    const TLE1: &str = "1 58023U 23155H   26161.48719104  .00004557  00000+0  19207-3 0  9996";
    const TLE2: &str = "2 58023  97.5689 243.0215 0003014  75.7594 284.3977 15.23841600147334";

    fn sample_app() -> App {
        let prop = Propagator::from_tle(TLE1, TLE2).unwrap();
        let t0 = Utc.with_ymd_and_hms(2026, 6, 10, 21, 21, 0).unwrap();
        let mut samples = Vec::new();
        for k in 0..10i64 {
            let t = t0 + Duration::seconds(k * 15);
            let ang = k as f64 * 0.05;
            let q =
                UnitQuaternion::from_quaternion(Quaternion::new(ang.cos(), ang.sin(), 0.0, 0.0));
            samples.push(AttitudeSample { t, q });
        }
        let track = Track {
            samples,
            convention: Convention::BodyToInertial,
        };
        let geom = Geometry {
            target_ecef_km: lla_to_ecef(51.2, 16.16, 0.0) / 1000.0,
            boresight_body: Vector3::new(1.0, 0.0, 0.0),
            carrier_hz: Some(1_296_000_000.0),
        };
        App::build(
            &track,
            &prop,
            &geom,
            t0,
            vec![("app-start".to_string(), t0)],
            vec![Window {
                start_s: 20.0,
                end_s: 40.0,
            }],
            "Legnica".into(),
            10.0,
        )
        .unwrap()
    }

    fn rendered_text(app: &App) -> String {
        let mut term = Terminal::new(TestBackend::new(140, 42)).unwrap();
        term.draw(|f| draw(f, app)).unwrap();
        term.backend()
            .buffer()
            .content()
            .iter()
            .map(|c| c.symbol())
            .collect()
    }

    #[test]
    fn main_view_renders() {
        let text = rendered_text(&sample_app());
        assert!(text.contains("boresight"));
        assert!(text.contains("readouts"));
    }

    #[test]
    fn table_view_renders_both_tables() {
        let mut app = sample_app();
        app.show_table = true;
        let text = rendered_text(&app);
        assert!(text.contains("per-window analysis"));
        assert!(text.contains("whole pass"));
    }
}
