# RUNBOOK — dcnt1-only (SolidContact) contact postprocessing

## Goal

Make the model and pipeline use **only** the Solid Mechanics **SolidContact** node `dcnt1` and remove/disable the obsolete **Contact** node `cnt1`, so that:
- Contact variables are defined on the intended contact region (`pc.destination.entities`)
- Mechanical `metrics.csv` and RP both use the same dcnt1-only contact definitions

## Why we弃用 `cnt1` (obsolete)

`solid.feature('cnt1')` is the legacy contact implementation and has proven to be a long-term maintenance risk:
- It can mask configuration problems (e.g. wrong contact-pair binding for `dcnt1`)
- It produces a separate family of contact fields that diverge from the intended “new-style” SolidContact path

Long-term requirement: **dcnt1-only**.

## Important COMSOL detail (this repo’s model)

Even when the SolidContact feature tag is `dcnt1`, the model exposes **unprefixed** contact fields:
- `solid.Tn`
- `solid.incontact`
- `solid.gap`

In this setup, `solid.dcnt1.*` can be **undefined** even while `dcnt1` exists and is active.

So “dcnt1-only” in practice means:
- enforce the **feature** `dcnt1` is bound to pair `pc`
- compute contact metrics from the **fields** `solid.Tn / solid.incontact / solid.gap`
- ensure `cnt1` is removed/disabled so those fields come from SolidContact, not obsolete contact

## How to repair an MPH (no commits of *.mph)

Use `repair_model_dcnt1_only.m` on any baseline MPH (for example `/mnt/data/Pyramid_5x5_checkpoint_last_ok.mph` in other environments):

```powershell
matlab -batch "repair_model_dcnt1_only('C:\\path\\to\\Pyramid_5x5_checkpoint_last_ok.mph')"
```

It writes a repaired model to:
- `out/pyramid_5x5/repair_dcnt1_only_YYYYMMDD_HHMMSS/Pyramid_5x5_repaired_dcnt1_only.mph`

## Metrics definition (dcnt1-only)

All contact metrics are evaluated on `pc.destination.entities`:

- `Tn_max_Pa = max(solid.Tn)`
- `Ac_m2` priority:
  1) `Ac = ∫ if(solid.incontact > 0.5, 1, 0) dA` (preferred)
  2) `Ac = ∫ if(solid.Tn > Tn_eps, 1, 0) dA`
  3) `Ac = ∫ if(solid.gap < 0, 1, 0) dA`
- `pn_avg_Pa = (∫ solid.Tn dA) / Ac` (when `Ac>0`)

## How to validate (diagnosis)

Run the dcnt1-only diagnosis on a solved sim directory:

```powershell
matlab -batch "diagnose_ac_from_sim_dir_dcnt1_only('out/pyramid_5x5/sim_YYYYMMDD_HHMMSS/')"
```

Expect in `out/pyramid_5x5/diag_ac_YYYYMMDD_HHMMSS/Ac_diagnose.csv`:
- `solid.incontact` / `solid.Tn` / `solid.gap` have `nonNaN_count > 0`
- `Ac_candidate_m2` is finite and (post-onset) typically `> 0`

