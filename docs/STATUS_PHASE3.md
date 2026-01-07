# STATUS_PHASE3 — Ac extraction diagnosis

## Update (2026-01-07 14:51) — Diagnose `Ac_m2=0` root cause (no new mechanical solves)

Inputs:
- Mechanical output (validated success run):
  - `out/pyramid_5x5/sim_20260107_133015/`
  - last_success: `1.0227` (early_exit=true)
  - force at last row (from metrics): `Fz_plate_top_int_N ≈ -5.69067e-06 N`
- Diagnosis output:
  - `out/pyramid_5x5/diag_ac_20260107_145744/Ac_diagnose.csv`

### Required checks

1) **Is `solid.dcnt1.Tn` evaluable at the solved state?**
- Result: **NO**
- Evidence (from `Ac_diagnose.csv`):
  - `solid.dcnt1.Tn`:
    - `nonNaN_count = 0`
    - `min/max = NaN/NaN`
    - error: `Undefined variable comp1.solid.dcnt1.Tn`
  - `solid.dcnt1.gap` similarly undefined.

2) **Does the integration selection `pc.destination` cover destination boundaries?**
- Result: **YES (non-empty)**
- Evidence:
  - selection used: `pc.destination.entities`
  - `selection_boundary_count = 126` (IDs preview in `Ac_diagnose.csv`)

3) **Candidate variables (cnt1/dcnt1/contact status) and Ac candidates**
- `solid.cnt1.Tn`:
  - `nonNaN_count = 755`
  - `min ≈ -4.89e-13`, `max ≈ 2.89e4` (Pa)
  - `Ac_candidate_m2 (Tn>0) ≈ 7.04e-09`
  - note: threshold `0` is likely too permissive; should use a positive threshold (e.g. 1–10 Pa) or `incontact`.
- `solid.cnt1.incontact`:
  - `nonNaN_count = 755`
  - `min=0`, `max=1`
  - `Ac_candidate_m2 (incontact>0.5) ≈ 2.324e-10`
- `solid.cnt1.gap`:
  - `nonNaN_count = 100/3846`
  - `min ≈ -1.09e-3`, `max ≈ 9.11e-4`
  - `Ac_candidate_m2 (gap<0) ≈ 1.32e-11`

### Conclusion (must choose A or B)

**A) Variable/selection/expression issue (fix postprocessing)**
- Root cause: current pipeline uses `solid.dcnt1.Tn` for `Ac_m2`, but in this 5×5 solved state `solid.dcnt1.*` is **undefined** while `solid.cnt1.*` is defined.
- Therefore `Ac_m2` in `out/pyramid_5x5/sim_20260107_133015/Pyramid_5x5_metrics.csv` stayed `0`, causing RP `Rc/R_total` to become `NaN` and `valid_points_for_RP=0`.

### Next-step suggestions (A/B routes)

Route A (recommended): fix Ac extraction (no physics changes)
- Change:
  - switch `Ac_m2` to be computed from `solid.cnt1.incontact` (preferred) or `solid.cnt1.Tn` with a positive threshold.
  - keep integrating on `pc.destination.entities`.
- Validate:
  - recompute/emit `Ac_m2>0` for the existing solved checkpoint(s) and then re-run RP; expect `valid_points_for_RP >= 1`.

Route B: continue pushing delta (only if Route A proves wrong)
- Change:
  - extend post_onset micro continuation beyond `1.0227` (e.g. `1.03/1.04`) to grow contact area.
- Validate:
  - mechanical `metrics.csv` contains at least one row where `Ac_m2>0` (under a correct and stable Ac extraction definition).
