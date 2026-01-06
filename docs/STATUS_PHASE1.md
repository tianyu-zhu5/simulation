# STATUS_PHASE1 (2026-01-06)

## Summary of changes vs phase0.5
- Added fail-fast controls in run_pyramid_array_5x5_sim_runbook.m: max_bisect_levels, min_indent_step_um, per_solve_timeout_s, global_timeout_s.
- Added fallback_report.json (records every attempt) and td_bridge_results.csv (records TD bridge attempt data).
- Added onset-cross TD bridge fallback (short transient ramp between last_ok and failed target).
- Updated RP pipeline (run_pyramid_array_5x5_rp.m) to use solid.dcnt1.Tn conventions and to emit summary diagnostics even when coverage is insufficient.
- .gitignore now includes MATLAB autosave (*.asv, *.m~).

## Mechanical runner (Phase 1) results
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Latest output directory:
- out/pyramid_5x5/sim_20260106_184312/

Artifacts present:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json
- fallback_report.json
- td_bridge_results.csv

Key outcomes (from summary):
- last_success_delta_total_um: 1.015
- fail_delta_total_um: 1.02
- narrow_fail_interval_um: [1.015, 1.02]
- bisect_levels_used: 0 (TD bridge attempted immediately at gap=0.005 um)
- fail_reason: td_bridge_failed (transient solver could not find consistent initial values)

Fail-fast settings recorded in summary:
- max_bisect_levels: 12
- min_indent_step_um: 0.001
- per_solve_timeout_s: 240
- global_timeout_s: 900
- onset_bridge_max_gap_um: 0.005

## RP pipeline (dcnt1 alignment) results
Run command:
- matlab -batch "run_pyramid_array_5x5_rp"

Latest output directory:
- out/pyramid_5x5/rp_20260106_191121/

Artifacts present:
- RP_raw.csv
- RP_interp.csv
- RP_summary.txt

Key outcomes (from RP_summary.txt):
- valid_points_for_RP: 0
- interp_status: not_enough_data
- last_solve_error: stationary solver nonconvergence (relative step too small)

## Notes
- Phase 1 achieved structured failure artifacts and TD-bridge attempt; the bridge did not cross the onset wall in this run.
- baseline_results.csv at repo root updated with fail_delta_total_um=1.02 and td_bridge_failed reason for all target deltas.
