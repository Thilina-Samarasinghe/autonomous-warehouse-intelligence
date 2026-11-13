# Results

All numbers below can be reproduced with the commands shown. Each experiment uses the same seeds for every planner, so every planner sees an identical task stream and identical robot spawn positions. Comparisons are therefore *paired by seed* (Wilcoxon signed-rank test). Effect sizes are Cliff's delta (|d| ≥ 0.474 = large).

**Common setup:** default 29×18 layout (192 shelves, 3 drop stations, 4 chargers), Zipf demand (s = 1.0), load held at 0.015 tasks/tick per robot, 3,000 ticks with a 300-tick warm-up, 10 seeds per cell. Idle robots on a station or pick point drive to a parking cell (an engine rule that applies to every planner).

"Gridlocked" = zero deliveries in each of the last 5 windows (500 ticks).

---

## 1. Main experiment: independent vs coordinated planning (phase 4)

```bash
cd backend
uv run awi-exp run scenarios/experiments/comparison.yaml --workers 6     # ~28 min on a 4-core laptop
```

| Robots | Planner | Throughput / 1k ticks (95% CI) | Avg delivery time | P95 delivery | Collisions* | Distance / delivery | Idle | Gridlocked runs |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 10 | Independent A* | 145.9 [142.1, 149.7] | 84.1 | 160.8 | 2,224 | 46.0 | 18% | 0/10 |
| 10 | Prioritized | 148.4 [143.5, 153.3] | 48.4 | 85.3 | **0** | 40.2 | 33% | 0/10 |
| 10 | **CBS** | **149.7** [144.8, 154.7] | **35.9** | **56.8** | **0** | **34.9** | 46% | 0/10 |
| 20 | Independent A* | 173.0 [104.2, 241.8] | 290.4 | 497.4 | 11,279 | 77.8 | 1% | **3/10** |
| 20 | Prioritized | 280.6 [274.6, 286.7] | 131.3 | 220.9 | **0** | 56.3 | 4% | 0/10 |
| 20 | **CBS** | **282.8** [278.1, 287.6] | **123.5** | 207.5 | **0** | 55.5 | 5% | 0/10 |
| 30 | Independent A* | 61.7 [16.1, 107.2] | 537.7 | 1,105.4 | 25,553 | 259.0 | 0% | **7/10** |
| 30 | Prioritized | 357.6 [350.3, 364.9] | 342.0 | 555.1 | **0** | 68.4 | 1% | 0/10 |
| 30 | **CBS** | **358.6** [350.8, 366.4] | **335.4** | 549.0 | **0** | 68.2 | 1% | 0/10 |

\* *Would-be collisions*: vertex and edge conflicts in the robots' intended moves, counted per tick, before the engine's safety arbitration stops them.

### The special-feature comparison: 10 robots, independent vs coordinated (CBS)

| Metric | Independent A* | CBS | Change | Wilcoxon p | Cliff's d |
|---|---:|---:|---:|---:|---:|
| Delivery time (mean, ticks) | 84.1 | 35.9 | **−57%** | 0.002 | −1.00 |
| Collisions (conflict-ticks) | 2,224 | **0** | **−100%** | 0.002 | −1.00 |
| Distance per delivery (cells) | 46.0 | 34.9 | **−24%** | 0.002 | −1.00 |
| Idle time (no task) | 18% | 46% | +29 pp | 0.002 | +0.98 |
| Throughput / 1k ticks | 145.9 | 149.7 | +2.6% | 0.004 | +0.37 |

At 10 robots the fleet has spare capacity, so throughput is capped by the order rate (both serve almost every order). The difference shows in **how** orders are served: CBS delivers in less than half the time, drives a quarter less, and never produces a conflict. The higher idle share is the *good* kind: robots finish faster and wait for the next order, instead of standing blocked in aisles.

### Findings

1. **Coordination removes collisions entirely.** Both coordinated planners produced **0 conflicts and 0 blocked moves** in all 60 runs, with zero planning failures. The independent fleet produced 2.2k to 25.5k conflict-ticks per run.
2. **Coordination is what makes the fleet scale.** Independent planning *loses* throughput as robots are added (146 → 173 → 62) and gridlocks in 7/10 runs at 30 robots. Coordinated planning scales almost linearly (148 → 281 → 358, about 2.4× from 10 to 30 robots). **At 30 robots, CBS delivers 5.8× more than independent A*** (p = 0.002, d = 1.0).
3. **CBS beats Prioritized Planning only at small fleets.** At 10 robots CBS is 26% faster per delivery (p = 0.002, d = −0.96) and drives 13% less. At 20 to 30 robots the two are statistically indistinguishable (p > 0.09). The reason is CBS's fallback rate: plain CBS exceeds its 100-node budget in **92% of replans at 20 robots and 98% at 30**, and then falls back to PP. This is the known exponential limit of optimal CBS (Sharon et al., 2015), so bounded-suboptimal ECBS would be the next step.
4. **The price of coordination is CPU time.** Mean planning time per replan was 0.2 ms for independent A*, 2 to 33 ms for PP, and 12 to 256 ms for CBS. PP gives about 99% of CBS's benefit at 10 to 15× less CPU. For live operation at 30 robots it is the practical choice.

---

## 2. Independent-planning baseline (phase 3, re-run with parking)

```bash
uv run awi-exp run scenarios/experiments/baseline_sweep.yaml --workers 6
```

| Robots | Planner | Throughput / 1k (mean, min–max) | Completion | Deadlock events | Blocked time | Gridlocked |
|---:|---|---|---:|---:|---:|---:|
| 10 | Independent A* | 145.9 (135.6–154.8) | 96% | 0.0 | 14% | 0/10 |
| 10 | Independent BFS | 145.7 (134.8–156.3) | 96% | 0.0 | 13% | 0/10 |
| 20 | Independent A* | 173.0 (0.0–261.1) | 57% | 11.9 | 39% | 3/10 |
| 20 | Independent BFS | 131.3 (20.7–253.0) | 43% | 4.3 | 50% | 5/10 |
| 30 | Independent A* | 61.7 (0.0–179.6) | 14% | 34.0 | 74% | 7/10 |
| 30 | Independent BFS | 97.4 (0.0–283.0) | 22% | 46.1 | 66% | 5/10 |

1. **Independent planning collapses instead of slowing down gradually.** At 20 to 30 robots a run either flows or locks up completely, once the aisle around a hot shelf fills from both ends. Hence the huge min–max spread.
2. **The single-agent search algorithm doesn't matter.** A* and BFS both find optimal single-agent paths, and the differences between them are noise. What's missing is *coordination*.
3. **Parking helps every planner.** Before the parking rule, independent A* at 10 robots averaged 99.7 ticks per delivery and was blocked 20% of the time. With parking: 84.1 ticks and 14%.
