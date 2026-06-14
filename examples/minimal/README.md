# Example: minimal input

The smallest input `boresight` needs: an attitude CSV, a TLE, and a target.
This example leans on every default, so the command stays short.

## Files

- `attitude.csv`: uses the default column names `time, qx, qy, qz, qw`, so no
  `--time-col` / `--quat-cols` are needed.
- `sat.tle`: the two element lines, read with `--tle-file`.

## Run

```bash
cargo build --release
examples/minimal/run.sh
```

which is just:

```bash
boresight \
  --attitude attitude.csv \
  --tle-file sat.tle \
  --target-lat 51.208333 --target-lon 16.160278
```

With no `--reference`, t=0 is the first attitude sample. With no `--windows`
the banner omits the window indicator and the chart has no window markers; the
boresight scope, ground track, and timeline still work. Add `--windows`,
`--marker`, `--reference`, `--boresight`, or `--carrier-hz` as the pass needs them.
