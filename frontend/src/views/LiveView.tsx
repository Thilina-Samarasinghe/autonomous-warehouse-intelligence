import { useEffect, useState } from 'react'
import { api } from '../api/client'
import type { Cell, Options, SessionStatus } from '../api/types'
import { useLiveSession } from '../api/useLiveSession'
import { KpiTiles } from '../components/KpiTiles'
import { LiveCharts } from '../components/LiveCharts'
import { MapLegend } from '../components/MapLegend'
import { RobotTable } from '../components/RobotTable'
import { ScenarioForm, StrategySelect } from '../components/ScenarioForm'
import { DEFAULT_SCENARIO, PRETTY, toConfig, type Scenario } from '../lib/scenario'
import { WarehouseCanvas } from '../components/WarehouseCanvas'

const SPEEDS = [2, 5, 10, 20, 50, 100, 200]

export function LiveView({ options }: { options: Options }) {
  const [scenario, setScenario] = useState<Scenario>(DEFAULT_SCENARIO)
  const [planner, setPlanner] = useState('prioritized')
  const [assigner, setAssigner] = useState('hungarian')
  const [session, setSession] = useState<SessionStatus | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [speed, setSpeed] = useState(10)
  const [blockTool, setBlockTool] = useState(false)
  const [showTable, setShowTable] = useState(false)
  const live = useLiveSession(session?.id ?? null)
  const running = live.frame?.running ?? false

  useEffect(
    () => () => {
      if (session) void api.deleteSession(session.id).catch(() => undefined)
    },
    [session],
  )

  const create = async () => {
    setError(null)
    try {
      setSession(
        await api.createSession({
          planner,
          assigner,
          speed,
          name: `${PRETTY[planner]} + ${PRETTY[assigner]}`,
          config: toConfig(scenario),
        }),
      )
    } catch (e) {
      setError(String(e instanceof Error ? e.message : e))
    }
  }

  const blockCell = async (cell: Cell) => {
    if (!session || !blockTool) return
    try {
      await api.addObstacles(session.id, [cell], 200)
      if (!running) live.send({ action: 'step', ticks: 1 })
    } catch (e) {
      setError(String(e instanceof Error ? e.message : e))
    }
  }

  return (
    <div className="view">
      <aside className="sidebar panel">
        <h2>Scenario</h2>
        <StrategySelect label="Path planner" options={options.planners} value={planner} onChange={setPlanner} />
        <StrategySelect label="Task assigner" options={options.assigners} value={assigner} onChange={setAssigner} />
        <ScenarioForm value={scenario} onChange={setScenario} />
        <button className="primary" onClick={() => void create()}>
          {session ? 'Restart with these settings' : 'Create simulation'}
        </button>
        {error && <p className="error">{error}</p>}
        <p className="sub">
          Robots, orders, people and blockages come from the seed, so the same seed replays the same
          day for every strategy.
        </p>
      </aside>

      <main className="main">
        {!session || !live.layout ? (
          <section className="panel empty">
            <h2>Live warehouse</h2>
            <p>Pick a planner and an assigner, then create a simulation.</p>
          </section>
        ) : (
          <>
            <section className="panel">
              <div className="toolbar">
                <div className="title">
                  <h2>{session.name}</h2>
                  <span className="sub">
                    tick {live.frame?.t ?? 0} · {session.num_robots} robots · {live.connection}
                  </span>
                </div>
                <div className="controls">
                  {running ? (
                    <button onClick={() => live.send({ action: 'pause' })}>Pause</button>
                  ) : (
                    <button className="primary" onClick={() => live.send({ action: 'start', speed })}>
                      Run
                    </button>
                  )}
                  <button disabled={running} onClick={() => live.send({ action: 'step', ticks: 1 })}>
                    Step
                  </button>
                  <button disabled={running} onClick={() => live.send({ action: 'step', ticks: 100 })}>
                    +100
                  </button>
                  <button onClick={() => live.send({ action: 'reset' })}>Reset</button>
                  <label className="inline">
                    Speed
                    <select
                      value={speed}
                      onChange={(e) => {
                        const v = Number(e.target.value)
                        setSpeed(v)
                        live.send({ action: 'speed', speed: v })
                      }}
                    >
                      {SPEEDS.map((s) => (
                        <option key={s} value={s}>
                          {s} ticks/s
                        </option>
                      ))}
                    </select>
                  </label>
                  <label className="check inline">
                    <input type="checkbox" checked={blockTool} onChange={(e) => setBlockTool(e.target.checked)} />
                    Click map to block an aisle
                  </label>
                </div>
              </div>
              <WarehouseCanvas
                layout={live.layout}
                frame={live.frame}
                onCellClick={blockTool ? (c) => void blockCell(c) : undefined}
              />
              <MapLegend />
            </section>
            <KpiTiles frame={live.frame} battery={scenario.battery} />
            <LiveCharts history={live.history} frame={live.frame} />
            <section className="panel">
              <div className="toolbar">
                <h3>Fleet table</h3>
                <button onClick={() => setShowTable((v) => !v)}>{showTable ? 'Hide' : 'Show'}</button>
              </div>
              {showTable && <RobotTable frame={live.frame} />}
            </section>
          </>
        )}
      </main>
    </div>
  )
}
