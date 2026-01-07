# RUNBOOK — RP blocker: `valid_points_for_RP=0` because `Ac_m2=0`

Validated RP output directory:
- `out/pyramid_5x5/rp_20260107_142038/`

Validated mechanical input directory:
- `out/pyramid_5x5/sim_20260107_133015/`
  - mechanical reached `last_success_delta_total_um=1.0227` and `early_exit=true`
  - force integral at the micro targets is on the order of `|Fz_top_int_N| ≈ 5e-6 N`
  - but `Ac_m2=0` and `Tn_max_Pa=NaN` (contact field undefined/guarded)

---

## 1) What happened (chain of causality)

RP computes:
- `P_eff = |Fz_top_int_N| / A_top`, where `A_top = (100um)^2 = 1e-8 m^2`
- series resistance:
  - `rho_c = rho_A * t_cnt` (Ω·m²)
  - `Rc = rho_c / Ac`
  - `R_total = R1 + R2 + Rc`

In the validated RP run (`out/pyramid_5x5/rp_20260107_142038/`):
- `valid_points_for_RP = 0`
- `interp_status = not_enough_data`

Reason:
- mechanical `Ac_m2` is **always 0** (no positive contact-pressure area detected)
- therefore `Rc = rho_c / 0` is treated as `NaN` (by our guard)
- therefore `R_total` becomes `NaN`
- mask requires `isfinite(P_eff) & isfinite(R_total) & P_eff>0`, so **zero valid points**

---

## 2) Contact criteria (layered) — what to trust at which stage

We need robust indicators across onset where `solid.dcnt1.Tn` may be undefined.

Recommended layered checks (SI units):

### L0 — Force onset (most robust)
- `|Fz_top_int_N| > 1e-9 N` ⇒ `contact_onset_detected=true`
  - In `sim_20260107_133015`, `|Fz| ≈ 5e-6 N` ⇒ onset is strongly indicated.

### L1 — Effective pressure
- `P_eff = |Fz| / A_top`
- With `A_top=1e-8 m^2`, `|Fz|=5e-6 N` ⇒ `P_eff ≈ 500 Pa = 0.5 kPa`
  - This lies inside the RP target window `[0.28, 1.0] kPa`.

### L2 — Contact area (required for Rc)
- `Ac_m2 > 0` is required to compute `Rc` and `R_total`.
- Proposed threshold for “valid contact point”:
  - `Ac_m2 > 0` **and** `isfinite(Ac_m2)`
  - optionally require `Ac_m2 > 1e-15 m^2` (avoid numerical dust).

### L3 — Contact pressure field
- `Tn_max_Pa` (from contact normal pressure) should become finite and positive once the contact field is defined.
- If `Tn_max_Pa=NaN` but `|Fz|` is large: treat as “onset but field undefined”, not “no contact”.

---

## 3) Two next-step routes (choose one)

### Route A) Keep pushing mechanical delta until `Ac_m2>0`

Goal:
- produce at least one mechanical row with `Ac_m2>0` (then RP will have valid points).

What to change:
- Only continuation targets/scheduling (no geometry/mesh changes):
  - extend micro targets beyond `1.0227` to e.g. `1.03` / `1.04`
  - keep `PTC-first` (`SIM_POST_SKIP_STATIONARY=1`) and a sufficient post budget

How to validate:
- In the new mechanical output dir:
  - `Pyramid_5x5_metrics.csv` contains at least one row where `Ac_m2>0`
  - then re-run RP and confirm:
    - `valid_points_for_RP >= 1`
    - `R_total_ohm` finite for at least one row

Risk:
- If `Ac_m2` remains 0 while force continues to grow, the “area extraction” is likely wrong (see Route B).

### Route B) Fix contact-area extraction (`Ac_m2`) / selection / expression

Goal:
- make `Ac_m2` reflect the true contacting area once force indicates contact.

What to check (in code + model):
1) Selection:
   - confirm the integration selection matches the **destination** boundaries on pair `pc`
2) Variable availability:
   - confirm whether `solid.dcnt1.Tn` is defined in this model at the solved state
   - current symptom: `Tn_max_Pa=NaN` even when `|Fz|≈5e-6 N`
3) Expression robustness:
   - if the contact pressure field is undefined at onset, use a guarded expression (example idea):
     - `Ac = ∫ if(isfinite(Tn) && Tn>0, 1, 0) dA`
   - if the variable name is wrong for the active contact feature, update to the correct variable and document it.

How to validate:
- Mechanical `Pyramid_5x5_metrics.csv` must show:
  - `Ac_m2 > 0` at some delta near where `|Fz|` is already O(1e-6 N)
- RP then must show:
  - `valid_points_for_RP >= 1`
  - `interp_status` can reach `ok` once `[0.28,1.0]kPa` is covered with finite `R_total`.

