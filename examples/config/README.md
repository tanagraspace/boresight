# Example: single config file

The whole pass is described in one TOML file, `pass.toml`, so the command is
just `--config` with no other flags. The file overrides the built-in defaults,
and an explicitly-passed flag in turn overrides the file. Relative paths
(`attitude`, `tle_file`) resolve against the config file's directory.

## Files

- `pass.toml`: attitude CSV, column names, TLE file, target, reference time,
  carrier, boresight, windows, and a marker, all in one place.
- `attitude.csv`, `sat.tle`: the data the config points at.

## Run

```bash
cargo build --release
examples/config/run.sh                 # interactive
examples/config/run.sh --dump-table    # or a non-interactive preview
```

which is just:

```bash
boresight --config examples/config/pass.toml
```

Any flag still overrides the file, e.g. `--config pass.toml --playback 4` to
start faster.
