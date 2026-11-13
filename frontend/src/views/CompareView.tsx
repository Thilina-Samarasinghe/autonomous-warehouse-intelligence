import { useEffect, useMemo, useRef, useState } from 'react'
import type Plotly from 'plotly.js-dist-min'
import { api } from '../api/client'
import type { Frame, Kpis, Options, SessionStatus } from '../api/types'
import { useLiveSession, type LiveSession } from '../api/useLiveSession'
import { MapLegend } from '../components/MapLegend'
import { ChartLegend } from '../components/ChartLegend'
import { PlotlyChart } from '../components/PlotlyChart'
import { ScenarioForm, StrategySelect } from '../components/ScenarioForm'
import { DEFAULT_SCENARIO, DEMO_PRESETS, PRETTY, toConfig, type Scenario } from '../lib/scenario'
import { WarehouseCanvas } from '../components/WarehouseCanvas'
import { cssVar } from '../lib/robotStates'
import { useThemeVersion } from '../lib/useTheme'

const FPS = 15
const SPEEDS = [5, 10, 20, 50, 100]

interface Side {
  planner: string
  assigner: string
}

const ROWS: { key: keyof Kpis | 'delivered'; label: string; better: 'high' | 'low'; digits: number }[] = [
  { key: 'delivered', label: 'Delivered', better: 'high', digits: 0 },
  { key: 'throughput_per_1k', label: 'Throughput / 1k ticks', better: 'high', digits: 1 },
  { key: 'delivery_time_mean', label: 'Avg delivery time', better: 'low', digits: 1 },
  { key: 'delivery_time_p95', label: 'P95 delivery time', better: 'low', digits: 1 },
  { key: 'conflicts_total', label: 'Would-be collisions', better: 'low', digits: 0 },
  { key: 'blocked_frac', label: 'Blocked time (share)', better: 'low', digits: 3 },
  { key: 'deadlock_events', label: 'Deadlock events', better: 'low', digits: 0 },
  { key: 'plan_ms_mean', label: 'Planner ms / replan', better: 'low', digits: 2 },
]

function value(f: Frame | null, key: keyof Kpis | 'delivered'): number | null {
  if (!f) return null
  return key === 'delivered' ? f.tasks.delivered : (f.kpis[key] as number | null)
}

/**
 * Side-by-side comparison on the *same* seed. Both simulations are stepped in
 * lockstep from the browser (step A and B, wait for both frames, repeat), so
 * they always show the same tick.
 */
export function CompareView({ options }: { options: Options }) {
  const [scenario, setScenario] = useState<Scenario>(DEFAULT_SCENARIO)
  const [preset, setPreset] = useState('')
  const [sides, setSides] = useState<[Side, Side]>([
    { planner: 'independent_astar', assigner: 'hungarian' },
    { planner: 'prioritized', assigner: 'hungarian' },
  ])
  const [sessions, setSessions] = useState<[SessionStatus, SessionStatus] | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [running, setRunning] = useState(false)
  const [speed, setSpeed] = useState(20)
  const a = useLiveSession(sessions?.[0].id ?? null)
  const b = useLiveSession(sessions?.[1].id ?? null)
  const theme = useThemeVersion()
  const runRef = useRef(false)

  useEffect(
    () => () => {
      sessions?.forEach((s) => void api.deleteSession(s.id).catch(() => undefined))
    },
    [sessions],
  )

  // Lockstep loop.
  const requestA = a.request
  const requestB = b.request
  useEffect(() => {
    runRef.current = running
    if (!running) return
    let timer = 0
    const ticks = Math.max(1, Math.round(speed / FPS))
    const loop = async () => {
      if (!runRef.current) return
      const started = performance.now()
      try {
        await Promise.all([requestA({ action: 'step', ticks }), requestB({ action: 'step', ticks })])
      } catch {
        setRunning(false)
        return
      }
      const wait = Math.max(0, (1000 * ticks) / speed - (performance.now() - started))
      timer = window.setTimeout(() => void loop(), wait)
    }
    void loop()
    return () => {
      runRef.current = false
      window.clearTimeout(timer)
    }
  }, [running, speed, requestA, requestB])

  const create = async () => {
    setError(null)
    setRunning(false)
    try {
      const config = toConfig(scenario)
      const made = await Promise.all(
        sides.map((s) =>
          api.createSession({ ...s, config, name: `${PRETTY[s.planner]} + ${PRETTY[s.assigner]}` }),
        ),
      )
      setSessions([made[0], made[1]])
    } catch (e) {
      setError(String(e instanceof Error ? e.message : e))
    }
  }

  const legendItems = [
    { label: `A · ${PRETTY[sides[0].planner]} + ${PRETTY[sides[0].assigner]}`, token: '--series-1' },
    { label: `B · ${PRETTY[sides[1].planner]} + ${PRETTY[sides[1].assigner]}`, token: '--series-2' },
  ]

  const reset = async () => {
    setRunning(false)
    await Promise.all([a.request({ action: 'reset' }), b.request({ action: 'reset' })]).catch(() => undefined)
  }

  const chart = useMemo(() => {
    void theme
    const trace = (
      history: LiveSession['history'],
      pick: (p: LiveSession['history'][number]) => number,
      color: string,
      name: string,
    ): Plotly.Data => ({
      type: 'scatter',
      mode: 'lines',
      x: history.map((p) => p.t),
      y: history.map(pick),
      name,
      line: { color, width: 2 },
      hovertemplate: `%{y:,.0f}<extra>${name}</extra>`,
    })
    const nameA = PRETTY[sides[0].planner]
    const nameB = PRETTY[sides[1].planner]
    const c1 = cssVar('--series-1')
    const c2 = cssVar('--series-2')
    const delivered = (p: LiveSession['history'][number]) => p.delivered
    const collided = (p: LiveSession['history'][number]) => p.conflicts
    return {
      deliveries: [trace(a.history, delivered, c1, `A · ${nameA}`), trace(b.history, delivered, c2, `B · ${nameB}`)],
      conflicts: [trace(a.history, collided, c1, `A · ${nameA}`), trace(b.history, collided, c2, `B · ${nameB}`)],
    }
  }, [a.history, b.history, sides, theme])

  return (
    <div className="view">
      <aside className="sidebar panel">
        <h2>Comparison</h2>
        <p className="sub">Two strategies, the same warehouse, the same orders, in lockstep.</p>
        <label>
          Demo scenario
          <select
            value={preset}
            onChange={(e) => {
              const p = DEMO_PRESETS.find((d) => d.id === e.target.value)
              setPreset(e.target.value)
              if (p) {
                setScenario(p.scenario)
                setSides([{ ...p.sides[0] }, { ...p.sides[1] }])
              }
            }}
          >
            <option value="">Custom</option>
            {DEMO_PRESETS.map((d) => (
              <option key={d.id} value={d.id}>
                {d.label}
              </option>
            ))}
          </select>
        </label>
        {preset && <p className="sub story">{DEMO_PRESETS.find((d) => d.id === preset)?.story}</p>}
        {(['A', 'B'] as const).map((label, i) => (
          <div className="side-block" key={label}>
            <h3>
              <span className="swatch dot" style={{ background: `var(--series-${i + 1})` }} /> Strategy {label}
            </h3>
            <StrategySelect
              label="Path planner"
              options={options.planners}
              value={sides[i].planner}
              onChange={(v) => setSides((s) => (i === 0 ? [{ ...s[0], planner: v }, s[1]] : [s[0], { ...s[1], planner: v }]))}
            />
            <StrategySelect
              label="Task assigner"
              options={options.assigners}
              value={sides[i].assigner}
              onChange={(v) => setSides((s) => (i === 0 ? [{ ...s[0], assigner: v }, s[1]] : [s[0], { ...s[1], assigner: v }]))}
            />
          </div>
        ))}
        <ScenarioForm value={scenario} onChange={setScenario} />
        <button className="primary" onClick={() => void create()}>
          {sessions ? 'Restart comparison' : 'Create comparison'}
        </button>
        {error && <p className="error">{error}</p>}
      </aside>

      <main className="main">
        {!sessions || !a.layout || !b.layout ? (
          <section className="panel empty">
            <h2>Independent vs coordinated</h2>
            <p>Pick two strategies and create the comparison. The defaults are the project's main experiment.</p>
          </section>
        ) : (
          <>
            <section className="panel">
              <div className="toolbar">
                <div className="title">
                  <h2>Tick {a.frame?.t ?? 0}</h2>
                  <span className="sub">seed {scenario.seed} · {scenario.robots} robots</span>
                </div>
                <div className="controls">
                  {running ? (
                    <button onClick={() => setRunning(false)}>Pause</button>
                  ) : (
                    <button className="primary" onClick={() => setRunning(true)}>
                      Run both
                    </button>
                  )}
                  <button disabled={running} onClick={() => void Promise.all([a.request({ action: 'step', ticks: 100 }), b.request({ action: 'step', ticks: 100 })])}>
                    +100
                  </button>
                  <button onClick={() => void reset()}>Reset</button>
                  <label className="inline">
                    Speed
                    <select value={speed} onChange={(e) => setSpeed(Number(e.target.value))}>
                      {SPEEDS.map((s) => (
                        <option key={s} value={s}>
                          {s} ticks/s
                        </option>
                      ))}
                    </select>
                  </label>
                </div>
              </div>
              <div className="pair">
                {[a, b].map((live, i) => (
                  <div key={i} className="pair-item">
                    <h3>
                      <span className="swatch dot" style={{ background: `var(--series-${i + 1})` }} />
                      {sessions[i].name}
                    </h3>
                    {live.layout && <WarehouseCanvas layout={live.layout} frame={live.frame} maxCell={18} />}
                  </div>
                ))}
              </div>
              <MapLegend />
            </section>

            <section className="panel">
              <h3>Head to head</h3>
              <table className="compare">
                <thead>
                  <tr>
                    <th>Metric</th>
                    <th className="num">
                      <span className="swatch dot" style={{ background: 'var(--series-1)' }} />A
                    </th>
                    <th className="num">
                      <span className="swatch dot" style={{ background: 'var(--series-2)' }} />B
                    </th>
                    <th className="num">B vs A</th>
                  </tr>
                </thead>
                <tbody>
                  {ROWS.map((row) => {
                    const va = value(a.frame, row.key)
                    const vb = value(b.frame, row.key)
                    let delta = '–'
                    let cls = ''
                    if (va !== null && vb !== null && va !== 0) {
                      const rel = (vb - va) / Math.abs(va)
                      delta = `${rel > 0 ? '+' : ''}${(rel * 100).toFixed(0)}%`
                      const good = row.better === 'high' ? rel > 0 : rel < 0
                      cls = Math.abs(rel) < 0.005 ? '' : good ? 'delta-good' : 'delta-bad'
                    }
                    const f = (v: number | null) =>
                      v === null ? '–' : v.toLocaleString(undefined, { maximumFractionDigits: row.digits })
                    return (
                      <tr key={row.key}>
                        <td>{row.label}</td>
                        <td className="num">{f(va)}</td>
                        <td className="num">{f(vb)}</td>
                        <td className={`num ${cls}`}>{delta}</td>
                      </tr>
                    )
                  })}
                </tbody>
              </table>
            </section>

            <div className="chart-grid">
              <section className="panel">
                <h3>Deliveries</h3>
                <p className="sub">Cumulative orders delivered</p>
                <ChartLegend items={legendItems} />
                <PlotlyChart empty={a.history.length < 2} data={chart.deliveries} layout={{ xaxis: { title: { text: 'Tick' } } }} ariaLabel="Deliveries, strategy A vs B" />
              </section>
              <section className="panel">
                <h3>Would-be collisions</h3>
                <p className="sub">Conflicts the engine had to stop (cumulative)</p>
                <ChartLegend items={legendItems} />
                <PlotlyChart empty={a.history.length < 2} data={chart.conflicts} layout={{ xaxis: { title: { text: 'Tick' } } }} ariaLabel="Collisions, strategy A vs B" />
              </section>
            </div>
          </>
        )}
      </main>
    </div>
  )
}
