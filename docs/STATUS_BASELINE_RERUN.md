# STATUS_BASELINE_RERUN (2026-01-06)

Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260106_174657/

Artifacts produced:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv (header + 14 data rows; includes ramp partials)
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json

Solve outcome (from summary/errors.json):
- last_success_delta_total_um: 1.0227
- fail_delta_total_um: 1.02275390625
- fail_reason: stationary solver nonconvergence (relative step too small) at contact onset

Partial metrics overview:
- metrics rows cover delta_total from 0 to 1.0227 um; contact metrics (Tn_max/Ac) stayed NaN/0 while Fz_plate became nonzero near 1.02 um.
- no target delta_total values (1.1 to 1.35 um) were reached.

Baseline results:
- baseline_results.csv updated at repo root with the 7 target deltas; all rows marked success=0 with fail_delta_total_um embedded in message.
