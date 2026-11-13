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
