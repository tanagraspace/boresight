# Example: Legnica pass, 2026-06-10

A real pass of OPS-SAT PRETTY (NORAD 58023) over Legnica, Poland, used as a
ready-to-run dataset for `boresight`. It is a high-elevation pass, peaking near
73 deg, with the spacecraft tracking the target throughout the captures.

## Files

- `attitude.csv`: UKF attitude telemetry (`time, ukf_X x, ukf_X y, ukf_X z, ukf_X k`),
  body-to-inertial quaternion, scalar-last.
- `run.sh`: launches `boresight` with this pass's TLE, target, reference time,
  an `experiment start` marker, carrier, and windows.

## Run

```bash
cargo build --release
examples/legnica-2026-06-10/run.sh
```

Extra flags pass through, for example a non-interactive preview:

```bash
examples/legnica-2026-06-10/run.sh --dump-table
examples/legnica-2026-06-10/run.sh --snapshot 160x48
```
