import type { Frame } from '../api/types'
import { STATE_GROUPS, groupOf, type StateGroup } from './robotStates'

export interface HistoryPoint {
  t: number
  delivered: number
  pending: number
  conflicts: number
  throughput: number | null
  groups: Record<StateGroup, number>
}

const MAX_POINTS = 1500

/** Bounded per-tick series built from streamed frames (one point per new tick). */
export class History {
  private data: HistoryPoint[] = []

  add(frame: Frame): void {
    const last = this.data[this.data.length - 1]
    if (last && frame.t < last.t) this.data = [] // session was reset
    if (last && frame.t === last.t) this.data.pop()
    const groups = Object.fromEntries(STATE_GROUPS.map((g) => [g.key, 0])) as Record<StateGroup, number>
    for (const r of frame.robots) groups[groupOf(r.state)] += 1
    this.data.push({
      t: frame.t,
      delivered: frame.tasks.delivered,
      pending: frame.tasks.pending,
      conflicts: frame.kpis.conflicts_total,
      throughput: frame.kpis.throughput_per_1k,
      groups,
    })
    if (this.data.length > MAX_POINTS) {
      // Keep every other point of the older half: long runs stay cheap to chart.
      const half = Math.floor(this.data.length / 2)
      this.data = [...this.data.slice(0, half).filter((_, i) => i % 2 === 0), ...this.data.slice(half)]
    }
  }

  points(): HistoryPoint[] {
    return [...this.data]
  }
}
