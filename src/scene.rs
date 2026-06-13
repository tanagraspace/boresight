//! The two left-pane views drawn into ratatui Canvases:
//!   - a ground track over a regional coastline map, and
//!   - a boresight error scope (polar, in degrees).

use nalgebra::Vector3;
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::Span;
use ratatui::widgets::canvas::{Context, Line as CLine, Map, MapResolution, Points};

use crate::app::App;

const COL_COAST: Color = Color::Rgb(70, 80, 96);
const COL_TRAJ_SUN: Color = Color::Rgb(214, 178, 32); // gold
const COL_TRAJ_SHADOW: Color = Color::Rgb(110, 110, 120);
const COL_TARGET: Color = Color::Rgb(60, 230, 90);
const COL_CAP: Color = Color::Rgb(90, 200, 230);
const COL_CITY: Color = Color::Rgb(150, 150, 162);
const COL_RING: Color = Color::Rgb(70, 74, 92);
const COL_AXISLBL: Color = Color::Rgb(120, 120, 130);

/// Major world capitals (lat, lon, name) drawn for geographic reference. Only
/// those inside the current map view render; a `+` marks the exact location and
/// the name sits to its right.
const CAPITALS: &[(f64, f64, &str)] = &[
    (51.51, -0.13, "London"),
    (48.86, 2.35, "Paris"),
    (52.52, 13.40, "Berlin"),
    (40.42, -3.70, "Madrid"),
    (41.90, 12.50, "Rome"),
    (52.23, 21.01, "Warsaw"),
    (50.45, 30.52, "Kyiv"),
    (55.76, 37.62, "Moscow"),
    (59.91, 10.75, "Oslo"),
    (59.33, 18.07, "Stockholm"),
    (60.17, 24.94, "Helsinki"),
    (55.68, 12.57, "Copenhagen"),
    (52.37, 4.90, "Amsterdam"),
    (50.85, 4.35, "Brussels"),
    (48.21, 16.37, "Vienna"),
    (50.08, 14.44, "Prague"),
    (47.50, 19.04, "Budapest"),
    (44.43, 26.10, "Bucharest"),
    (37.98, 23.73, "Athens"),
    (39.93, 32.85, "Ankara"),
    (38.72, -9.14, "Lisbon"),
    (53.35, -6.26, "Dublin"),
    (46.95, 7.45, "Bern"),
    (38.91, -77.04, "Washington"),
    (45.42, -75.70, "Ottawa"),
    (19.43, -99.13, "Mexico City"),
    (-15.79, -47.88, "Brasilia"),
    (-34.60, -58.38, "Buenos Aires"),
    (-12.05, -77.04, "Lima"),
    (4.71, -74.07, "Bogota"),
    (-33.45, -70.67, "Santiago"),
    (30.04, 31.24, "Cairo"),
    (-1.29, 36.82, "Nairobi"),
    (-25.75, 28.19, "Pretoria"),
    (9.06, 7.50, "Abuja"),
    (9.03, 38.74, "Addis Ababa"),
    (34.02, -6.84, "Rabat"),
    // Africa
    (14.72, -17.47, "Dakar"),
    (0.39, 9.45, "Libreville"),
    (12.37, -1.52, "Ouagadougou"),
    (5.56, -0.20, "Accra"),
    (36.75, 3.06, "Algiers"),
    (36.81, 10.18, "Tunis"),
    (32.89, 13.19, "Tripoli"),
    (15.50, 32.56, "Khartoum"),
    (12.64, -8.00, "Bamako"),
    (13.51, 2.11, "Niamey"),
    (18.08, -15.98, "Nouakchott"),
    (12.13, 15.06, "N'Djamena"),
    (4.39, 18.56, "Bangui"),
    (6.82, -5.28, "Yamoussoukro"),
    (6.13, 1.22, "Lome"),
    (6.50, 2.62, "Porto-Novo"),
    (3.85, 11.50, "Yaounde"),
    (-4.32, 15.31, "Kinshasa"),
    (-4.27, 15.28, "Brazzaville"),
    (-8.84, 13.23, "Luanda"),
    (-15.42, 28.28, "Lusaka"),
    (-17.83, 31.05, "Harare"),
    (-25.97, 32.57, "Maputo"),
    (-6.16, 35.75, "Dodoma"),
    (0.35, 32.58, "Kampala"),
    (-1.94, 30.06, "Kigali"),
    (2.05, 45.32, "Mogadishu"),
    (-18.88, 47.51, "Antananarivo"),
    (-22.57, 17.08, "Windhoek"),
    (-24.65, 25.91, "Gaborone"),
    (35.69, 51.39, "Tehran"),
    (33.31, 44.36, "Baghdad"),
    (24.71, 46.68, "Riyadh"),
    (24.45, 54.38, "Abu Dhabi"),
    (28.61, 77.21, "New Delhi"),
    (35.68, 139.65, "Tokyo"),
    (37.57, 126.98, "Seoul"),
    (39.90, 116.41, "Beijing"),
    (13.76, 100.50, "Bangkok"),
    (-6.21, 106.85, "Jakarta"),
    (14.60, 120.98, "Manila"),
    (1.35, 103.82, "Singapore"),
    (-35.28, 149.13, "Canberra"),
    (-41.29, 174.78, "Wellington"),
];

// ---- Ground track ---------------------------------------------------------

/// Sub-satellite point of an ECEF position, as (lon_deg, lat_deg). Geocentric
/// latitude, which is within ~0.2 deg of geodetic and fine for a track plot.
pub fn subpoint(ecef_km: Vector3<f64>) -> (f64, f64) {
    let lon = ecef_km.y.atan2(ecef_km.x).to_degrees();
    let hyp = (ecef_km.x * ecef_km.x + ecef_km.y * ecef_km.y).sqrt();
    let lat = ecef_km.z.atan2(hyp).to_degrees();
    (lon, lat)
}

/// Bounding box (lon_min, lon_max, lat_min, lat_max) over the whole track and
/// the target, before any padding or aspect correction.
pub fn ground_data_bbox(app: &App) -> (f64, f64, f64, f64) {
    let (mut lon0, mut lon1, mut lat0, mut lat1) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    let mut acc = |lon: f64, lat: f64| {
        lon0 = lon0.min(lon);
        lon1 = lon1.max(lon);
        lat0 = lat0.min(lat);
        lat1 = lat1.max(lat);
    };
    for f in &app.frames {
        let (lon, lat) = subpoint(f.sc_ecef_km);
        acc(lon, lat);
    }
    let (tlon, tlat) = subpoint(app.target_ecef_km);
    acc(tlon, tlat);
    (lon0, lon1, lat0, lat1)
}

/// Map view bounds (`x = [lon_min, lon_max]`, `y = [lat_min, lat_max]`) for a
/// data bounding box `(lon_min, lon_max, lat_min, lat_max)`, given the cell
/// aspect `va` and a zoom factor. Pads the data, enforces a minimum span so
/// surrounding land shows, aspect-corrects by `cos(lat)` so coastlines are not
/// stretched, applies zoom, and clamps to valid lon/lat.
pub fn fit_region(bbox: (f64, f64, f64, f64), va: f64, zoom: f64) -> ([f64; 2], [f64; 2]) {
    let (lon0, lon1, lat0, lat1) = bbox;
    let clon = (lon0 + lon1) / 2.0;
    let clat = (lat0 + lat1) / 2.0;
    let mut lat_span = ((lat1 - lat0) * 1.3).max(22.0);
    let mut lon_span = ((lon1 - lon0) * 1.3).max(10.0);
    let k = va / clat.to_radians().cos().abs().max(0.1);
    let lon_needed = lat_span * k;
    if lon_needed >= lon_span {
        lon_span = lon_needed;
    } else {
        lat_span = lon_span / k;
    }
    lat_span /= zoom;
    lon_span /= zoom;
    let x = [
        (clon - lon_span / 2.0).max(-180.0),
        (clon + lon_span / 2.0).min(180.0),
    ];
    let y = [
        (clat - lat_span / 2.0).max(-90.0),
        (clat + lat_span / 2.0).min(90.0),
    ];
    (x, y)
}

/// The visible map region plus its cell dimensions, used to place labels and to
/// declutter them by occupancy.
pub struct MapView {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
    pub cols: u16,
    pub rows: u16,
}

impl MapView {
    /// Cell column/row of a lon/lat, or None if outside the view.
    fn cell(&self, lon: f64, lat: f64) -> Option<(i32, i32)> {
        let fx = (lon - self.xmin) / (self.xmax - self.xmin);
        let fy = (self.ymax - lat) / (self.ymax - self.ymin);
        if !(0.0..=1.0).contains(&fx) || !(0.0..=1.0).contains(&fy) {
            return None;
        }
        Some((
            (fx * self.cols as f64) as i32,
            (fy * self.rows as f64) as i32,
        ))
    }
}

/// Tracks which label cells are taken so overlapping names can be dropped.
struct Occupancy {
    cols: i32,
    rows: i32,
    taken: Vec<bool>,
}

impl Occupancy {
    fn new(cols: u16, rows: u16) -> Self {
        Self {
            cols: cols as i32,
            rows: rows as i32,
            taken: vec![false; cols as usize * rows as usize],
        }
    }

    /// Try to place a `len`-cell label anchored at (col, row), keeping a margin
    /// of `pad_x` columns and `pad_y` rows clear around it. The anchor must be in
    /// bounds; the margin box is clamped to the grid. Returns true (and reserves
    /// the box) if every cell in the box was free.
    fn try_place(&mut self, col: i32, row: i32, len: i32, pad_x: i32, pad_y: i32) -> bool {
        if row < 0 || row >= self.rows || col < 0 || col + len > self.cols {
            return false;
        }
        let c0 = (col - pad_x).max(0);
        let c1 = (col + len + pad_x).min(self.cols);
        let r0 = (row - pad_y).max(0);
        let r1 = (row + pad_y + 1).min(self.rows);
        for r in r0..r1 {
            for c in c0..c1 {
                if self.taken[(r * self.cols + c) as usize] {
                    return false;
                }
            }
        }
        for r in r0..r1 {
            for c in c0..c1 {
                self.taken[(r * self.cols + c) as usize] = true;
            }
        }
        true
    }
}

/// Label spacing margin (pad_x cols, pad_y rows) as a function of the view's
/// latitude span: the more zoomed out, the larger the keep-out box, so fewer
/// labels survive and the map stays readable.
fn declutter_pad(span_lat: f64) -> (i32, i32) {
    match span_lat {
        s if s < 15.0 => (1, 0),
        s if s < 35.0 => (2, 1),
        s if s < 70.0 => (4, 1),
        _ => (8, 2),
    }
}

pub fn paint_groundtrack(ctx: &mut Context, app: &App, view: &MapView) {
    ctx.draw(&Map {
        resolution: MapResolution::High,
        color: COL_COAST,
    });

    // Track, colored by sunlit / shadow.
    for i in 1..app.frames.len() {
        let (x1, y1) = subpoint(app.frames[i - 1].sc_ecef_km);
        let (x2, y2) = subpoint(app.frames[i].sc_ecef_km);
        let color = if app.frames[i].sunlit {
            COL_TRAJ_SUN
        } else {
            COL_TRAJ_SHADOW
        };
        ctx.draw(&CLine {
            x1,
            y1,
            x2,
            y2,
            color,
        });
    }

    // Window-start subpoints.
    for &idx in &app.window_start_idx {
        let (x, y) = subpoint(app.frames[idx].sc_ecef_km);
        ctx.draw(&Points {
            coords: &[(x, y)],
            color: COL_CAP,
        });
    }

    // Reference labels: '+' at the exact location, name to its right. The target
    // claims its cells first (highest priority), then capitals in list order; a
    // label is dropped when its keep-out box is already taken. The box grows as
    // the view zooms out, thinning labels more aggressively at small scales.
    // `l` hides the capitals entirely.
    let mut occ = Occupancy::new(view.cols, view.rows);
    let (pad_x, pad_y) = declutter_pad(view.ymax - view.ymin);

    let (tlon, tlat) = subpoint(app.target_ecef_km);
    if let Some((c, r)) = view.cell(tlon, tlat) {
        let len = app.target_name.chars().count() as i32 + 1;
        occ.try_place(c, r, len, 1, 0);
        ctx.print(
            tlon,
            tlat,
            Span::styled(
                format!("+{}", app.target_name),
                Style::default().fg(COL_TARGET).add_modifier(Modifier::BOLD),
            ),
        );
    }

    if app.show_labels {
        for &(lat, lon, name) in CAPITALS {
            let Some((c, r)) = view.cell(lon, lat) else {
                continue;
            };
            let len = name.chars().count() as i32 + 1;
            if occ.try_place(c, r, len, pad_x, pad_y) {
                ctx.print(
                    lon,
                    lat,
                    Span::styled(format!("+{name}"), Style::default().fg(COL_CITY)),
                );
            }
        }
    }

    // Current sub-satellite point: a bold asterisk colored by pointing accuracy
    // (green on target through amber to red), matching the boresight scope.
    let (slon, slat) = subpoint(app.current().sc_ecef_km);
    let sc_col = err_color(
        app.current().pointing_err_deg,
        scope_r_max(app.max_pointing_err_deg),
    );
    ctx.print(
        slon,
        slat,
        Span::styled(
            "*",
            Style::default().fg(sc_col).add_modifier(Modifier::BOLD),
        ),
    );
}

// ---- Boresight error scope -----------------------------------------------
//
// A polar view centred on the boresight axis. The target is plotted at a
// radius equal to the pointing error (degrees) and a clock bearing equal to the
// target's direction around the boresight, decomposed onto the body +Z (up) and
// +Y (right) axes. As the spacecraft slews on target the dot spirals to centre.

/// On-target color gradient: green when the error is small, amber in the
/// middle, red near the full-scale radius. `t` is the error as a fraction of
/// `r_max`. Gives a smooth transition as the boresight converges on target.
pub fn err_color(err_deg: f64, r_max: f64) -> Color {
    let t = (err_deg / r_max).clamp(0.0, 1.0);
    let lerp = |a: (f64, f64, f64), b: (f64, f64, f64), u: f64| {
        Color::Rgb(
            (a.0 + (b.0 - a.0) * u) as u8,
            (a.1 + (b.1 - a.1) * u) as u8,
            (a.2 + (b.2 - a.2) * u) as u8,
        )
    };
    let green = (60.0, 230.0, 90.0);
    let amber = (235.0, 195.0, 60.0);
    let red = (225.0, 70.0, 60.0);
    if t < 0.5 {
        lerp(green, amber, t / 0.5)
    } else {
        lerp(amber, red, (t - 0.5) / 0.5)
    }
}

/// Nice ring spacing (degrees) for a given full-scale radius.
fn ring_step(r_max: f64) -> f64 {
    match r_max {
        r if r <= 10.0 => 2.0,
        r if r <= 20.0 => 5.0,
        r if r <= 45.0 => 10.0,
        _ => 15.0,
    }
}

/// Full-scale radius of the scope in degrees, given the pass's maximum pointing
/// error: rounded up to a whole ring and floored so a near-zero pass still shows
/// useful rings.
pub fn scope_r_max(max_err_deg: f64) -> f64 {
    let base = max_err_deg.max(5.0);
    let step = ring_step(base);
    (base / step).ceil() * step
}

fn draw_circle(ctx: &mut Context, r: f64, col: Color) {
    let n = 64;
    let mut prev = (r, 0.0);
    for k in 1..=n {
        let a = std::f64::consts::TAU * k as f64 / n as f64;
        let p = (r * a.sin(), r * a.cos());
        ctx.draw(&CLine {
            x1: prev.0,
            y1: prev.1,
            x2: p.0,
            y2: p.1,
            color: col,
        });
        prev = p;
    }
}

pub fn paint_scope(ctx: &mut Context, app: &App, r_max: f64) {
    let f = app.current();

    // Work in the body frame. The line of sight, expressed in body coordinates,
    // is its projection onto the three body axes (which are stored in ECEF).
    let los_ecef = (app.target_ecef_km - f.sc_ecef_km).normalize();
    let los_body = Vector3::new(
        los_ecef.dot(&f.body_axes_ecef[0]),
        los_ecef.dot(&f.body_axes_ecef[1]),
        los_ecef.dot(&f.body_axes_ecef[2]),
    );

    // Boresight is the configured axis; pick two transverse reference axes for
    // the scope's up/right, perpendicular to it, so this works for any boresight.
    let bore = app.boresight_body;
    let world_ref = if bore.dot(&Vector3::new(0.0, 0.0, 1.0)).abs() < 0.9 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let up_axis = (world_ref - bore * world_ref.dot(&bore)).normalize();
    let right_axis = up_axis.cross(&bore); // +X boresight, +Z up -> +Y right

    let err = f.pointing_err_deg;
    let los_perp = los_body - bore * los_body.dot(&bore);
    let right = los_perp.dot(&right_axis);
    let up = los_perp.dot(&up_axis);
    let bearing = right.atan2(up);

    // Rings and their labels.
    let step = ring_step(r_max);
    let mut ring = step;
    while ring <= r_max + 1e-6 {
        draw_circle(ctx, ring, COL_RING);
        ctx.print(
            0.0,
            ring,
            Span::styled(format!("{ring:.0}"), Style::default().fg(COL_AXISLBL)),
        );
        ring += step;
    }

    // Cross-hair and body-axis labels.
    ctx.draw(&CLine {
        x1: -r_max,
        y1: 0.0,
        x2: r_max,
        y2: 0.0,
        color: COL_RING,
    });
    ctx.draw(&CLine {
        x1: 0.0,
        y1: -r_max,
        x2: 0.0,
        y2: r_max,
        color: COL_RING,
    });
    let lbl = |ctx: &mut Context, x: f64, y: f64, s: String| {
        ctx.print(x, y, Span::styled(s, Style::default().fg(COL_AXISLBL)));
    };
    lbl(ctx, 0.0, r_max, crate::pointing::axis_label(up_axis));
    lbl(
        ctx,
        r_max * 0.86,
        0.0,
        crate::pointing::axis_label(right_axis),
    );
    lbl(ctx, 0.0, -r_max, crate::pointing::axis_label(-up_axis));
    lbl(
        ctx,
        -r_max * 0.92,
        0.0,
        crate::pointing::axis_label(-right_axis),
    );

    // Target vector and dot, colored by how far off target we are.
    let px = err * bearing.sin();
    let py = err * bearing.cos();
    let col = err_color(err, r_max);
    ctx.draw(&CLine {
        x1: 0.0,
        y1: 0.0,
        x2: px,
        y2: py,
        color: col,
    });
    ctx.draw(&Points {
        coords: &[(0.0, 0.0)],
        color: Color::White,
    });
    ctx.draw(&Points {
        coords: &[(px, py)],
        color: col,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subpoint_known_points() {
        let (lon, lat) = subpoint(Vector3::new(6378.0, 0.0, 0.0));
        assert!(lon.abs() < 1e-6 && lat.abs() < 1e-6);
        let (_, lat_n) = subpoint(Vector3::new(0.0, 0.0, 6378.0));
        assert!((lat_n - 90.0).abs() < 1e-6);
        let (lon_e, _) = subpoint(Vector3::new(0.0, 6378.0, 0.0));
        assert!((lon_e - 90.0).abs() < 1e-6);
    }

    #[test]
    fn err_color_gradient_endpoints() {
        if let Color::Rgb(r, g, b) = err_color(0.0, 10.0) {
            assert!(g > r && g > b, "on target should be green");
        } else {
            panic!("expected rgb");
        }
        if let Color::Rgb(r, g, b) = err_color(10.0, 10.0) {
            assert!(r > g && r > b, "off target should be red");
        } else {
            panic!("expected rgb");
        }
    }

    #[test]
    fn ring_step_thresholds() {
        assert_eq!(ring_step(8.0), 2.0);
        assert_eq!(ring_step(18.0), 5.0);
        assert_eq!(ring_step(40.0), 10.0);
        assert_eq!(ring_step(100.0), 15.0);
    }

    #[test]
    fn scope_r_max_floors_and_rounds() {
        assert_eq!(scope_r_max(0.0), 6.0); // floor 5 -> ceil(5/2)*2
        assert_eq!(scope_r_max(13.6), 15.0); // step 5 -> ceil(13.6/5)*5
    }

    #[test]
    fn declutter_pad_grows_when_zoomed_out() {
        let (px_in, py_in) = declutter_pad(10.0);
        let (px_out, py_out) = declutter_pad(120.0);
        assert!(px_out > px_in && py_out >= py_in);
    }

    #[test]
    fn occupancy_overlap_and_padding() {
        let mut o = Occupancy::new(20, 5);
        assert!(o.try_place(0, 0, 3, 0, 0));
        assert!(!o.try_place(1, 0, 3, 0, 0), "overlapping label rejected");
        assert!(o.try_place(10, 0, 3, 0, 0), "far label accepted");
        // Padding reserves neighbours.
        let mut p = Occupancy::new(20, 5);
        assert!(p.try_place(5, 2, 1, 2, 1));
        assert!(
            !p.try_place(6, 2, 1, 0, 0),
            "within padded keep-out rejected"
        );
    }

    #[test]
    fn mapview_cell_in_and_out() {
        let v = MapView {
            xmin: 0.0,
            xmax: 10.0,
            ymin: 0.0,
            ymax: 10.0,
            cols: 10,
            rows: 10,
        };
        assert!(v.cell(5.0, 5.0).is_some());
        assert!(v.cell(-1.0, 5.0).is_none());
        assert!(v.cell(5.0, 20.0).is_none());
    }

    #[test]
    fn fit_region_min_span_and_zoom() {
        // Degenerate (single point) box -> minimum spans applied.
        let (_x, y) = fit_region((16.0, 16.0, 51.0, 51.0), 1.0, 1.0);
        let span = y[1] - y[0];
        assert!(span >= 21.9, "min lat span enforced, got {span}");
        // Zooming in shrinks the span.
        let (_x2, y2) = fit_region((16.0, 16.0, 51.0, 51.0), 1.0, 2.0);
        assert!((y2[1] - y2[0]) < span);
    }
}
