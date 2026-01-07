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

## Update (2026-01-06 21:44)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260106_212833/

Artifacts present:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json
- fallback_report.json
- ptc_bridge_results.csv

Key outcomes:
- last_success_delta_total_um: 1.02 (segmented stationary advanced beyond prior fail)
- fail_delta_total_um: 1.05
- fail_reason: Global timeout exceeded before solve: 958.3s > 900.0s
- bridge_policy: PTC>SEGMENTED>TD_RELAX
- bridge_mode: SEGMENTED (success)
- ptc_attempted: true (failed with relative step too small)
- td_attempted: false

Classification:
- PRIMARY: CONTACT_ONSET_STIFFNESS
- Evidence: stationary/ptc failures around onset + global timeout before next delta.

## Update (2026-01-06 22:44)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260106_224426/

Artifacts present:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json
- fallback_report.json
- ptc_bridge_results.csv

Key outcomes:
- last_success_delta_total_um: 1.0205
- fail_delta_total_um: 1.0205
- fail_reason: Global timeout exceeded after post_onset segmented: 903.7s > 900.0s
- delta_plan_mode_final: post_onset_micro (micro targets active, no 1.05 jump)
- bridge_policy: PTC>SEGMENTED>TD_RELAX
- ptc_attempted: true
- segmented_attempted: true

Classification:
- PRIMARY: CONTACT_ONSET_STIFFNESS
- Evidence: segmented needed near onset, global timeout hit during post_onset micro continuation.

## Update (2026-01-06 23:11)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260106_231141/

Artifacts present:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json
- fallback_report.json
- ptc_bridge_results.csv

Key outcomes:
- last_success_delta_total_um: 1.02
- fail_delta_total_um: 1.0205
- fail_reason: Global timeout exceeded before post_onset solve: 1082.3s > 900.0s
- delta_plan_mode_final: post_onset_micro (switch triggered)
- narrow_fail_interval_um: [1.02, 1.0205]
- bridge_mode: SEGMENTED (pre-onset)

Classification:
- PRIMARY: CONTACT_ONSET_STIFFNESS
- Evidence: segmented/PTC required to reach 1.02; global timeout before first micro target.

## Update (2026-01-06 23:44)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260106_234435/

Artifacts present:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json
- fallback_report.json
- ptc_bridge_results.csv

Key outcomes:
- last_success_delta_total_um: 1.0202
- fail_delta_total_um: 1.0205
- fail_reason: post_onset_micro_failed at 1.0205 um (relative step too small)
- delta_plan_mode_final: post_onset_micro (entered micro targets)
- narrow_fail_interval_um: [1.02, 1.0205]

Classification:
- PRIMARY: CONTACT_ONSET_STIFFNESS
- Evidence: micro-target segmented at 1.0205 still hit relative step too small.

## Update (2026-01-07 00:11) — Phase1.3 hard budget run
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260107_001133/

Artifacts present:
- Pyramid_5x5_summary.txt
- Pyramid_5x5_metrics.csv
- Pyramid_5x5_checkpoint_last_ok.mph
- errors.json
- fallback_report.json
- ptc_bridge_results.csv
- td_bridge_results.csv

Key outcomes:
- last_success_delta_total_um: 1.0183
- fail_delta_total_um: 1.02
- fail_reason: bridge_failed (relative step too small)
- delta_plan_mode_final: pre_onset (did not reach micro targets)
- narrow_fail_interval_um: [1.015, 1.02]

Classification:
- PRIMARY: CONTACT_ONSET_STIFFNESS
- Evidence: pre-onset segmented/PTC/TD all failed at 1.02; micro targets not reached.

Termination note (Phase1.3):
- Outcome: post_onset_micro could not reliably advance beyond 1.0205; second run failed to enter micro.
- Recommend Phase2 minimal upgrades:
  1) Switch contact algorithm preset (e.g., Augmented Lagrange or auto-penalty).
  2) Enable ν stabilization (e.g., ν=0.48 or nearly-incompressible formulation).

## Update (2026-01-07 13:03) — Phase2 resume micro continuation
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260107_130330/

Key outcomes:
- RESUME_POST_ONSET_ONLY: true (resume_from_dir=out/pyramid_5x5/sim_20260107_120640/)
- entered_post_onset_micro: true
- micro_target attempted: 1.0205 (PTC OK; stationary hit per_solve_timeout)
- last_success_delta_total_um: 1.0205
- fail_reason: post_onset budget exhausted before reaching 1.0227 (budget_post_onset_micro_s=300)

Classification:
- PRIMARY: CONTACT_ONSET_STIFFNESS
- Evidence: post_onset_micro requires slow PTC convergence; runtime consumed the post budget before reaching 1.0227.

## Update (2026-01-07 13:30) — Phase2 augmented_lagrange, resume micro reached 1.0227
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Output directory:
- out/pyramid_5x5/sim_20260107_133015/

Key outcomes:
- RESUME_POST_ONSET_ONLY: true (resume_from_dir=out/pyramid_5x5/sim_20260107_130330/)
- post_skip_stationary: true (PTC-first)
- last_success_delta_total_um: 1.0227
- early_exit: true (Reached Phase1 target >=1.0227)
- fail_reason: none
