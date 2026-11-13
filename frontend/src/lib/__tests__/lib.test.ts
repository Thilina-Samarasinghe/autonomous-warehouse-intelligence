import { describe, expect, it } from 'vitest'
import type { Frame, RobotStateName } from '../../api/types'
import { History } from '../history'
import { STATE_GROUPS, groupOf } from '../robotStates'
import { DEFAULT_SCENARIO, toConfig } from '../scenario'

function frame(t: number, states: RobotStateName[], delivered = 0): Frame {
  return {
    type: 'frame',
    session: 's',
    t,
    running: false,
    speed: 10,
    robots: states.map((state, id) => ({ id, pos: [0, id], state, goal: null, battery: null, carrying: 0, path: [] })),
    humans: [],
    blocked: [],
    tasks: { generated: delivered, pending: 0, delivered, in_progress: 0 },
    kpis: {
      throughput_per_1k: null,
      delivery_time_mean: null,
      delivery_time_p95: null,
      conflicts_total: 0,
      blocked_frac: 0,
      idle_frac: 0,
      charging_frac: 0,
      deadlock_events: 0,
      obstacle_blocks: 0,
      plan_ms_mean: 0,
      stranded_robots: 0,
    },
  }
}

describe('robot state groups', () => {
  it('maps every state to one of four groups', () => {
    expect(groupOf('to_pickup')).toBe('fetching')
    expect(groupOf('picking')).toBe('fetching')
    expect(groupOf('to_drop')).toBe('carrying')
    expect(groupOf('dropping')).toBe('carrying')
    expect(groupOf('to_charger')).toBe('charging')
    expect(groupOf('charging')).toBe('charging')
    expect(groupOf('idle')).toBe('free')
    expect(groupOf('to_park')).toBe('free')
    expect(STATE_GROUPS.map((g) => g.token)).toEqual(['--series-1', '--series-2', '--series-3', '--neutral-mark'])
  })
})

describe('History', () => {
  it('records one point per tick and counts state groups', () => {
    const h = new History()
    h.add(frame(1, ['idle', 'to_pickup']))
    h.add(frame(1, ['to_drop', 'to_pickup'], 1)) // same tick: replaced
    h.add(frame(2, ['charging', 'charging'], 2))
    const pts = h.points()
    expect(pts.map((p) => p.t)).toEqual([1, 2])
    expect(pts[0].groups).toEqual({ free: 0, fetching: 1, carrying: 1, charging: 0 })
    expect(pts[1].delivered).toBe(2)
  })

  it('starts over after a reset and stays bounded', () => {
    const h = new History()
    for (let t = 0; t < 4000; t++) h.add(frame(t, ['idle']))
    expect(h.points().length).toBeLessThanOrEqual(1500)
    expect(h.points().at(-1)?.t).toBe(3999)
    h.add(frame(0, ['idle']))
    expect(h.points().map((p) => p.t)).toEqual([0])
  })
})

describe('scenario -> SimConfig', () => {
  it('scales the order rate with the fleet and toggles subsystems', () => {
    const cfg = toConfig({ ...DEFAULT_SCENARIO, robots: 20, ratePerRobot: 0.015, people: 3, battery: true })
    expect(cfg.num_robots).toBe(20)
    expect(cfg.task_rate).toBeCloseTo(0.3)
    expect(cfg.battery).toEqual({ enabled: true })
    expect(cfg.dynamics).toEqual({ enabled: true, humans: 3, blockage_rate: 0 })
    expect((toConfig(DEFAULT_SCENARIO).dynamics as { enabled: boolean }).enabled).toBe(false)
  })
})
