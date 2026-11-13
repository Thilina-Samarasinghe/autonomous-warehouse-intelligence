/** Types mirroring the FastAPI backend (backend/src/awi/api). */

export type Cell = [number, number]

export type RobotStateName =
  | 'idle'
  | 'to_park'
  | 'to_pickup'
  | 'picking'
  | 'to_drop'
  | 'dropping'
  | 'to_charger'
  | 'charging'
  | 'stranded'

export interface Layout {
  type?: 'layout'
  width: number
  height: number
  cells: number[][]
  cell_types: Record<string, number>
  drop_stations: Cell[]
  chargers: Cell[]
  parking: Cell[]
}

export interface RobotFrame {
  id: number
  pos: Cell
  state: RobotStateName
  goal: Cell | null
  battery: number | null
  carrying: number
  path: Cell[]
}

export interface Kpis {
  throughput_per_1k: number | null
  delivery_time_mean: number | null
  delivery_time_p95: number | null
  conflicts_total: number
  blocked_frac: number
  idle_frac: number
  charging_frac: number
  deadlock_events: number
  obstacle_blocks: number
  plan_ms_mean: number
  stranded_robots: number
}

export interface Frame {
  type: 'frame'
  session: string
  t: number
  running: boolean
  speed: number
  robots: RobotFrame[]
  humans: Cell[]
  blocked: Cell[]
  tasks: { generated: number; pending: number; delivered: number; in_progress: number }
  kpis: Kpis
}

export interface SessionStatus {
  id: string
  name: string
  planner: string
  assigner: string
  t: number
  running: boolean
  speed: number
  num_robots: number
  config: Record<string, unknown>
}

export interface Options {
  planners: string[]
  assigners: string[]
  default_config: Record<string, unknown>
}

export interface SessionRequest {
  planner: string
  assigner: string
  name?: string
  speed?: number
  config: Record<string, unknown>
}

export type WsCommand =
  | { action: 'start'; speed?: number }
  | { action: 'pause' }
  | { action: 'reset' }
  | { action: 'step'; ticks: number }
  | { action: 'speed'; speed: number }
