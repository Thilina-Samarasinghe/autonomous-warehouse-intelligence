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

## 2. Task assignment strategies (phase 5)

```bash
uv run awi-exp run scenarios/experiments/assignment.yaml --workers 6     # ~5 min
```

**Setup:** prioritized planning (coordinated, 0 conflicts in every run), **heavier load** of 0.022 tasks/tick per robot, so that assignment decisions matter. Batching robots carry up to **2 items per trip**; all other assigners carry 1. 10 seeds per cell.

| Robots | Assigner | Throughput / 1k (95% CI) | Completion | Avg delivery | P95 delivery | Wait for a robot | Distance / delivery |
|---:|---|---|---:|---:|---:|---:|---:|
| 10 | Greedy (FIFO, Manhattan) | 191.4 [187.0, 195.8] | 86% | 215.0 | 363.2 | 168.8 | 42.6 |
| 10 | Hungarian | 200.8 [196.0, 205.5] | 91% | 155.3 | 278.1 | 110.3 | 39.8 |
| 10 | CP-SAT | 200.3 [195.9, 204.7] | 90% | 157.7 | 281.3 | 112.8 | 39.7 |
| 10 | **OR-Tools batching** | **216.8** [210.9, 222.7] | **98%** | **63.2** | **108.5** | **6.2** | **33.8** |
| 20 | Greedy | 271.6 [263.1, 280.1] | 61% | 606.5 | 997.4 | 551.6 | 59.2 |
| 20 | Hungarian | 280.4 [272.9, 287.8] | 63% | 580.8 | 957.3 | 526.7 | 56.6 |
| 20 | CP-SAT | 280.5 [272.7, 288.2] | 63% | 579.4 | 955.5 | 525.3 | 56.6 |
| 20 | **OR-Tools batching** | **418.0** [407.8, 428.2] | **94%** | **133.6** | **229.2** | **52.0** | **35.9** |

### Findings

1. **Optimal matching beats greedy.** Hungarian raises throughput by 4.9% and cuts delivery time by 28% at 10 robots (p = 0.002, d = 0.74), because it uses true travel distances and optimises all robots jointly instead of serving tasks FIFO.
2. **Hungarian and CP-SAT are interchangeable.** They reach the same optimum: a property test checks both against brute force, and they show no measurable difference on any KPI (p > 0.27). Individual runs differ only through tie-breaks between equally good matchings. CP-SAT is 10 to 15% slower, but it can take side constraints (eligibility, battery reserve, hard priorities) that Hungarian cannot express.
3. **Batching changes the game under load.** Carrying 2 items per trip, the fleet serves **54% more orders at 20 robots** (418 vs 272 per 1k ticks, p = 0.002, d = 1.0). Average delivery time drops by **78%** (607 → 134 ticks) and distance per delivery by 39%. The 1-item strategies cannot keep up with the order rate (61 to 63% completion, a growing backlog). Batching serves 94% of orders.

---

## 3. Battery management (phase 6)

```bash
uv run awi-exp run scenarios/experiments/battery.yaml --workers 6     # ~6 min
```

**Setup:** prioritized planning + Hungarian assignment, 6,000 ticks (500 warm-up) so every robot goes through several charge cycles, 10 seeds. Energy: 0.1 per move, +0.05 per item carried, 0.01 per tick idle, chargers add 1.0 per tick. Robots start between 50% and 100%. Policy: charge below 25% when free; emergency charge (cargo kept aboard) below 10% or when only just able to reach a charger; charge up to 90%. Variants: battery off, default (4 chargers), 2 chargers, and a conservative policy (charge below 40%, up to 95%).

| Robots | Variant | Throughput / 1k (95% CI) | Completion | Avg delivery | Time charging | Charge sessions | Energy / delivery | Lowest battery | Stranded |
|---:|---|---|---:|---:|---:|---:|---:|---:|---:|
| 10 | Battery off | 148.9 [146.7, 151.1] | 99% | 45.8 | — | — | — | — | — |
| 10 | 4 chargers | 148.9 [146.8, 151.1] | 99% | 49.3 | 8.5% | 66 | 5.6 | 15% | **0** |
| 10 | 2 chargers | 148.8 [146.8, 150.9] | 99% | 51.1 | 8.6% | 67 | 5.6 | 14% | **0** |
| 10 | Conservative | 148.9 [146.7, 151.0] | 99% | 50.2 | 8.4% | 76 | 5.6 | 29% | **0** |
| 20 | Battery off | 289.4 [283.6, 295.1] | 96% | 155.2 | — | — | — | — | — |
| 20 | 4 chargers | 272.8 [266.1, 279.5] | 90% | 316.8 | 10.4% | 156 | 7.5 | 6% | **0** |
| 20 | 2 chargers | 223.6 [216.3, 231.0] | 74% | 774.0 | 7.4% | 107 | 6.6 | 8% | **0** |
| 20 | Conservative | 271.7 [264.7, 278.6] | 90% | 319.7 | 10.2% | 179 | 7.5 | 16% | **0** |

### Findings

1. **No robot ever ran flat.** All 80 runs had 0 stranded robots and 0 conflicts. The fleet-wide lowest battery was 4.5%. Getting there took two policy fixes, which this experiment first exposed (an earlier version stranded up to 7 robots per run at 20 robots):
   - *Emergency charging with cargo aboard.* Under heavy traffic a delivery leg can take far longer than its shortest path (one leg took 192 ticks for about 15 cells), so the up-front energy estimate is not enough. A robot that becomes critical mid-delivery now charges first and resumes afterwards.
   - *Charger preemption.* An emergency robot may take the charger of a robot already charged above 25%.
2. **With spare capacity, energy management is free.** At 10 robots, throughput is unchanged (p = 0.94). Deliveries are only 3.5 ticks slower, from charging stops.
3. **Under load, chargers become the bottleneck.** At 20 robots, 4 chargers cost 5.7% throughput compared with infinite energy (p = 0.002). Halving to 2 chargers costs a further **18%** (p = 0.002, d = −1.0) and more than doubles delivery time, because robots queue for charging. The charger count must scale with fleet size; about 1 charger per 5 robots is enough here.
4. **A conservative threshold buys safety for free.** Charging below 40% instead of 25% raises the lowest battery seen from 6% to 16%, with no measurable throughput cost (p = 0.63). Robots charge more often (179 vs 156 sessions), but each session is shorter.
5. **Energy per delivery grows with congestion**: 5.6 at 10 robots vs 7.5 at 20. That's the energy cost of detours and of moving aside in a busier warehouse.

---

## 4. Independent-planning baseline (phase 3, re-run with parking)

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
