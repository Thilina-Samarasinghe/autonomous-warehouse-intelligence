/** Scenario form model shared by the Live and Compare views. */
export interface Scenario {
  robots: number
  seed: number
  ratePerRobot: number
  capacity: number
  battery: boolean
  people: number
  blockages: boolean
}

export const DEFAULT_SCENARIO: Scenario = {
  robots: 10,
  seed: 0,
  ratePerRobot: 0.015,
  capacity: 2,
  battery: false,
  people: 0,
  blockages: false,
}

/** Map the form to backend SimConfig overrides. */
export function toConfig(s: Scenario): Record<string, unknown> {
  return {
    num_robots: s.robots,
    seed: s.seed,
    task_rate: Number((s.ratePerRobot * s.robots).toFixed(4)),
    capacity: s.capacity,
    battery: { enabled: s.battery },
    dynamics: {
      enabled: s.people > 0 || s.blockages,
      humans: s.people,
      blockage_rate: s.blockages ? 0.01 : 0,
    },
  }
}

export const PRETTY: Record<string, string> = {
  independent_astar: 'Independent A*',
  independent_bfs: 'Independent BFS',
  prioritized: 'Prioritized Planning',
  cbs: 'CBS (Conflict-Based Search)',
  greedy: 'Greedy (FIFO)',
  hungarian: 'Hungarian',
  cpsat: 'OR-Tools CP-SAT',
  batching: 'OR-Tools batching',
}

export interface StrategyPair {
  planner: string
  assigner: string
}

export interface DemoPreset {
  id: string
  label: string
  story: string
  scenario: Scenario
  sides: [StrategyPair, StrategyPair]
}

/** One-click scenarios for live demos (Compare view). Each tells one result. */
export const DEMO_PRESETS: DemoPreset[] = [
  {
    id: 'main',
    label: 'Main experiment · 10 robots',
    story: 'Same throughput, but coordination removes every collision and delivers ~2x faster.',
    // Matches the main experiment exactly (greedy assignment, seed 0).
    scenario: { ...DEFAULT_SCENARIO, robots: 10 },
    sides: [
      { planner: 'independent_astar', assigner: 'greedy' },
      { planner: 'cbs', assigner: 'greedy' },
    ],
  },
  {
    id: 'stress',
    label: 'Scaling stress · 30 robots',
    story: 'Independent robots gridlock around tick 1,000 (deliveries stop for good); the coordinated fleet keeps flowing.',
    // Seed 2 of the main experiment: the independent fleet gridlocks earliest.
    scenario: { ...DEFAULT_SCENARIO, robots: 30, seed: 2 },
    sides: [
      { planner: 'independent_astar', assigner: 'greedy' },
      { planner: 'prioritized', assigner: 'greedy' },
    ],
  },
  {
    id: 'people',
    label: 'People in the aisles · 20 robots, 8 people',
    story: 'Robots keep a safety zone around people; the coordinated fleet replans around them.',
    scenario: { ...DEFAULT_SCENARIO, robots: 20, people: 8 },
    sides: [
      { planner: 'independent_astar', assigner: 'hungarian' },
      { planner: 'prioritized', assigner: 'hungarian' },
    ],
  },
  {
    id: 'batching',
    label: 'Order batching · 20 robots, heavy load',
    story: 'Carrying two items per trip clears a backlog that one-item trips cannot.',
    scenario: { ...DEFAULT_SCENARIO, robots: 20, ratePerRobot: 0.022, capacity: 2 },
    sides: [
      { planner: 'prioritized', assigner: 'greedy' },
      { planner: 'prioritized', assigner: 'batching' },
    ],
  },
  {
    id: 'battery',
    label: 'Battery management · 20 robots',
    story: 'Robots charge before they run flat; no robot is ever stranded.',
    scenario: { ...DEFAULT_SCENARIO, robots: 20, battery: true },
    sides: [
      { planner: 'independent_astar', assigner: 'hungarian' },
      { planner: 'prioritized', assigner: 'hungarian' },
    ],
  },
]
