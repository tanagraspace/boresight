# boresight

Animated terminal visualizer for spacecraft attitude and target pointing: a
ground track, a boresight error scope, and a synchronized pointing-error /
ground-station-elevation chart, driven by attitude telemetry and a TLE.

It replays a pass and shows, second by second, how well the spacecraft held its
aim at a ground target. The ground track, the boresight scope, and the chart
cursor share one timeline, so they move together as you play, scrub, or jump
between events.

## What it shows

```
┌ banner: UTC time · window n/m · sunlit/shadow · boresight-on-target ──────────┐
├───────────────────────────────────┬──────────────────────────────────────────┤
│ ground track (coastline map)       │ pointing error & elevation vs time        │
│  · sub-satellite track, gold        │  · red: boresight-to-target angle         │
│    sunlit / gray in eclipse         │  · blue: elevation from the target        │
│  · major capitals (+name), target   │  · faint verticals: windows               │
│    (+name), current sub-point (*)   │  · moving vertical: playback cursor       │
├───────────────────────────────────┤    (colored by pointing accuracy)         │
│ boresight scope (polar, degrees)    ├──────────────────────────────────────────┤
│  · target at radius = error,        │ Doppler / range-rate strip: signed curve  │
│    bearing around the boresight     │  crossing zero at closest approach        │
│  · green on target → amber → red    ├──────────────────────────────────────────┤
│                                     │ readouts: error, elevation, azimuth,      │
│                                     │ slant range, Doppler, sunlit, markers,    │
│                                     │ TLE epoch/age, playback state             │
└───────────────────────────────────┴──────────────────────────────────────────┘
```

The boresight scope is the precision view: the target sits at a radius equal to
the pointing error (concentric rings are degrees) and a clock bearing equal to
its direction around the boresight, decomposed onto two transverse body axes
(labeled for whichever boresight is configured). The dot, its vector, the
ground-track `*`, and the chart cursor are color-coded green when on target,
fading through amber to red as the error grows, so convergence reads at a glance.

Press `t` for the analysis tables: a per-window summary (start/end/min/mean
pointing angle, elevation range, Doppler-or-range-rate, sunlit fraction) and a
whole-pass table sampled at a step you can change with `[` / `]` (default ~10
rows).

## Build

```bash
cargo build --release
# binary at target/release/boresight
```

## Usage

The repo ships ready-to-run examples (`cargo build --release` first):

```bash
examples/config/run.sh             # everything from one config file
examples/minimal/run.sh            # minimum required input, all defaults
examples/legnica-2026-06-10/run.sh # the full set of input params and flags
# any of them takes --snapshot WxH for a non-interactive preview (--dump-table needs --windows)
```

Everything can come from a single TOML file:

```bash
boresight --config pass.toml
```

or from flags:

```bash
boresight \
  --attitude attitude.csv \
  --time-col time --quat-cols "qx,qy,qz,qw" \
  --tle-file sat.tle \
  --target-lat 51.208333 --target-lon 16.160278 --target-name Legnica \
  --reference 2026-06-10T21:21:20Z \
  --marker "app-start=2026-06-10T21:21:23Z" \
  --windows "9.9:29.9,33:53,55.8:75.8,79.4:99.4,102.4:122.4,126:146"
```

An explicitly-passed flag overrides the config file, so
`--config pass.toml --playback 4` runs everything from the file but at 4x speed.
The config file in turn overrides the built-in defaults. Three non-interactive
modes:

- `--dump-table` prints the per-window analysis table and exits.
- `--snapshot 160x48` renders one static frame as text at the given size and
  exits (useful for previews, docs, and CI).
- `--export-csv PATH` writes the whole-pass timeline (time, pointing error,
  elevation, azimuth, slant, range rate, Doppler if a carrier is set, sunlit,
  sub-satellite lat/lon) to a CSV file and exits. By default it writes one row
  per timeline frame; `--export-csv-step SECONDS` thins it to one row every
  SECONDS (rounded to the nearest frame).

## Inputs

- **Config** (`--config pass.toml`): bundles all of the below in one TOML file;
  see `examples/config`. Relative paths resolve against the file's directory.
- **Attitude CSV** (`--attitude`): one row per sample. The time column
  (`--time-col`, default `time`) is ISO-8601 (`Z`-terminated); the quaternion
  columns (`--quat-cols`, default `qx,qy,qz,qw`) are scalar-last `(x, y, z, w)`.
  Samples may be irregularly spaced; gaps are SLERP-interpolated.
- **TLE**: either the two lines inline via `--tle1` / `--tle2`, or a file via
  `--tle-file` (2LE, or 3LE with a leading name line). Propagated with SGP4; the
  tool shows the TLE epoch and its age relative to the reference time.
- **Target**: `--target-lat`/`--target-lon` in degrees, or `--target-ecef` in
  metres.
- **Boresight** (`--boresight "x,y,z"`, default `1,0,0` = +X): the body axis the
  pointing error and scope are measured against, i.e. whatever has to point at
  the target (an antenna, a camera, a thruster); any axis works (e.g. `0,-1,0`).
- **Reference time** (`--reference`): the t=0 origin for the chart and for
  window offsets. Defaults to the first attitude sample.
- **Markers** (`--marker LABEL=TIME`, repeatable): arbitrary labeled instants
  shown in the HUD as offsets from the reference and used as jump targets. The
  minimum-pointing-error instant is always computed and marked.
- **Windows** (`--windows`): `start:end,...` in seconds relative to the
  reference time; drawn on the chart and summarized in the per-window table.
- **Carrier** (`--carrier-hz`): optional. When set, the readouts and tables show
  Doppler; when omitted they show range rate (km/s) instead.
- **Playback** (`--playback`, default 1.0): initial speed multiplier; `0` starts
  paused.
- **Timeline step** (`--dt`, default 1.0): seconds between sampled frames; a
  smaller step gives a smoother timeline and more `--export-csv` rows.

## Keys

| Key | Action |
|---|---|
| `space` | play / pause |
| `←` / `→` | step one frame back / forward |
| `,` / `.` | jump to previous / next event (window start, elevation peak, min pointing error, eclipse exit, reference time, markers) |
| `0`–`9` | scrub to 0%–90% of the timeline |
| `-` / `+` | slower / faster playback |
| `r` | reset to start |
| `Home` / `End` | jump to first / last frame |
| `z` / `Z` | zoom the ground-track map in / out |
| `l` | toggle the capital labels on the ground track |
| `t` | toggle the analysis tables (per-window + whole-pass) |
| `[` / `]` | in the table view, coarser / finer whole-pass step (fewer / more rows) |
| `↑` / `↓` · `PgUp` / `PgDn` | in the table view, scroll the whole-pass table |
| `q` / `Esc` | quit |

## Development

```bash
cargo test          # unit tests across all modules
cargo clippy        # lints
cargo fmt           # format

# Render one frame headlessly (no terminal needed), handy for docs/CI:
examples/config/run.sh --snapshot 160x48
```

The code is split so the geometry is testable without a terminal: `astro`
(frames, GMST, Sun), `orbit` (SGP4 → ECEF), `pointing` (attitude loading,
interpolation, per-instant geometry, per-window stats), `app` (timeline, events,
playback), `scene` (ground-track and scope drawing), `ui` (layout), `config`
(the TOML file), and `main` (CLI).

## Conventions

- GMST: IAU-1982 / Meeus polynomial with full elapsed days.
- TEME (SGP4 output) to ECEF: rotation about Z by GMST only (polar motion and
  nutation ignored; sub-km at LEO).
- Quaternion is body-to-inertial, scalar-last; pass `--invert-quat` if your
  telemetry stores inertial-to-body instead.
- Boresight defaults to body +X; change it with `--boresight "x,y,z"`.
- Sunlit/shadow uses a cylindrical Earth-shadow model and a low-precision Sun
  ephemeris.

## License

MIT
