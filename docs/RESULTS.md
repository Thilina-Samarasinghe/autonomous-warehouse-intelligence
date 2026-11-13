# Results

All numbers below can be reproduced with the commands shown. Each experiment uses the same seeds for every planner, so every planner sees an identical task stream and identical robot spawn positions. Comparisons are therefore *paired by seed*.

## 1. Independent-planning baseline (phase 3)

```bash
cd backend
uv run awi-exp run scenarios/experiments/baseline_sweep.yaml --workers 6
```

**Setup:** default 29×18 layout (192 shelves, 3 drop stations), Zipf demand (s = 1.0), load held at 0.015 tasks/tick per robot, 3,000 ticks with a 300-tick warm-up, 10 seeds per cell. Planner: independent A* with the reactive detour/yield rules.

| Robots | Throughput / 1k ticks (mean, min–max) | Completion rate | Deadlock events | Blocked time | Runs gridlocked at the end |
|---:|---|---:|---:|---:|---:|
| 10 | **145.1** (133.7–155.6) | 96% | 0.0 | 20% | 0 / 10 |
| 20 | 141.1 (3.3–247.0) | 46% | 3.8 | 48% | **5 / 10** |
| 30 | 79.2 (0.0–194.4) | 18% | 16.4 | 71% | **7 / 10** |

*"Gridlocked at the end" = zero deliveries in each of the last 5 windows (500 ticks).*

**Findings**

1. **Independent planning does not scale.** Going from 10 to 20 robots doubles the offered load, yet mean throughput *falls*. At 30 robots it falls to about half the 10-robot level, because most runs end in permanent gridlock.
2. **The failure mode is collapse, not gradual slowdown.** The spread at 20 to 30 robots is huge (0 to 247 deliveries per 1k ticks). A run either flows or locks up completely, once the aisle around a hot shelf fills from both ends and nobody has room to back out.
3. **A* vs BFS makes no significant difference** to any fleet KPI (Wilcoxon p > 0.1 for throughput at every fleet size). Both find optimal single-agent paths, so the choice of single-agent search is irrelevant. *Coordination* is what is missing. A* costs more planning time at 30 robots (3.0 vs 1.0 ms per call, p = 0.002), because detour searches with many blocked cells weaken its heuristic.

These are the numbers that coordinated planning (CBS, phase 4) has to beat.
