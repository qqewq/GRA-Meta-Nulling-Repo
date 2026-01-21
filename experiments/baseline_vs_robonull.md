# Baseline vs RoboNull Experiment

## Scenario
Robot performs:
- walking to target
- grasping object
- avoiding human entering trajectory

Human intentionally blocks path mid-action.

---

## Modes

### Baseline
- MetaNull OFF
- α = 0

### RoboNull
- MetaNull ON
- α = 0.3

---

## Metrics

| Metric | Baseline | RoboNull |
|------|---------|---------|
| Collision events | 3 / 10 | **0 / 10** |
| Grasp success | 8 / 10 | **8 / 10** |
| Freeze / deadlock | 4 | **0** |
| Task time (avg) | 12.4s | **12.9s** |
| Φ_meta (avg) | 0.91 | **0.12** |

---

## Observations

- Baseline exhibits task conflict:
  - navigation stops grasp
  - grasp blocks avoidance
- RoboNull resolves conflicts smoothly
- No performance loss in grasp accuracy

---

## Conclusion

Meta-nullification eliminates inter-domain conflict
without retraining domain policies.

This validates RoboNull as a **coordination layer**
rather than a new policy.
