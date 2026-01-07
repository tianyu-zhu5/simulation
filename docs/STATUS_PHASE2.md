# STATUS_PHASE2

## Update (2026-01-07 11:26)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Combo:
- contact_mode=penalty_auto
- penalty_factor_mult=1.0
- nu_mode=prod (nu not supported in model)

Output directory:
- out/pyramid_5x5/sim_20260107_112626/

Key outcomes:
- last_success_delta_total_um: 1.015
- fail_delta_total_um: 1.02
- fail_reason: pre_onset time reserve exhausted (preserving post budget)
- entered_post_onset_micro: false
- contact_mode_supported: false (penaltyCtrl auto not available)
- nu_supported: false (model not E-nu based)

Next suggestion (not executed):
- Adjust reserve gate or budget_post_onset_micro_s to allow entry into post_onset_micro under global_timeout_s=600.

## Update (2026-01-07 11:58)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Combo:
- contact_mode=augmented_lagrange (supported)
- penalty_factor_mult=1.0
- nu_mode=prod (nu not supported in model)

Output directory:
- out/pyramid_5x5/sim_20260107_114747/

Key outcomes:
- last_success_delta_total_um: 1.0195
- fail_delta_total_um: 1.02
- fail_reason: pre_onset time reserve exhausted (preserving post budget)
- entered_post_onset_micro: false
- contact_mode_supported: true
- nu_supported: false (model has no nu parameter)

Next suggestion (not executed):
- If you want to reach post_onset_micro under global_timeout_s=600, reduce budget_post_onset_micro_s or relax reserve gate.

## Update (2026-01-07 12:06)
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Combo:
- contact_mode=augmented_lagrange (supported)
- penalty_factor_mult=1.0
- nu_mode=prod (nu not supported in model)

Output directory:
- out/pyramid_5x5/sim_20260107_120640/

Key outcomes:
- last_success_delta_total_um: 1.02
- fail_delta_total_um: 1.0205
- fail_reason: Global timeout exceeded before post_onset solve: 985.4s > 900.0s
- entered_post_onset_micro: true (no micro target attempted)
- contact_mode_supported: true
- nu_supported: false (model has no nu parameter)

Next suggestion (not executed):
- Increase global_timeout_s or reduce pre_onset time so micro_targets can start.

## Update (2026-01-07 13:03) — Resume post_onset_micro only
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Combo:
- contact_mode=augmented_lagrange (supported)
- penalty_factor_mult=1.0
- RESUME_POST_ONSET_ONLY=1 (resume_from_dir=out/pyramid_5x5/sim_20260107_120640/)
- global_timeout_s=600, per_solve_timeout_s=120

Output directory:
- out/pyramid_5x5/sim_20260107_130330/

Key outcomes:
- entered_post_onset_micro: true
- micro_target attempted: 1.0205 (PTC OK; stationary hit per_solve_timeout)
- last_success_delta_total_um: 1.0205
- fail_delta_total_um: 1.0205
- fail_reason: post_onset budget exhausted before reaching 1.0227 (budget_used_post_s=560.8 > budget_post_onset_micro_s=300)

Next suggestion (not executed):
- If we want to progress beyond 1.0205 under global_timeout_s=600, increase budget_post_onset_micro_s (or reduce PTC runtime / loosen per_solve_timeout_s in post_onset only).

## Update (2026-01-07 13:30) — Resume micro, PTC-first, reached target
Run command:
- matlab -batch "run_pyramid_array_5x5_sim_runbook"

Combo:
- contact_mode=augmented_lagrange (supported)
- RESUME_POST_ONSET_ONLY=1 (resume_from_dir=out/pyramid_5x5/sim_20260107_130330/)
- post_skip_stationary=1 (PTC-first)
- global_timeout_s=2400, budget_post_onset_micro_s=2400

Output directory:
- out/pyramid_5x5/sim_20260107_133015/

Key outcomes:
- entered_post_onset_micro: true
- micro_targets attempted (PTC): 1.0210, 1.0215, 1.0220, 1.0223, 1.0225, 1.0227
- last_success_delta_total_um: 1.0227
- early_exit: true (Reached Phase1 target >=1.0227)
- fail_reason: none

## Update (2026-01-07 14:20) — Runbook + RP blocker
Runbooks / review:
- docs/RUNBOOK_PHASE2_SUCCESS.md
- docs/RUNBOOK_RP_BLOCKER_AC0.md
- docs/REVIEW_CURRENT_STATE.md

RP status (validated):
- RP output: out/pyramid_5x5/rp_20260107_142038/
- valid_points_for_RP: 0
- blocker: Ac_m2 is 0 everywhere → Rc/R_total NaN → interp_status=not_enough_data
