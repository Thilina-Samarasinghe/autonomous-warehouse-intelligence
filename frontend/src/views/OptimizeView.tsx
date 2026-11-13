import { useMemo, useState } from 'react'
import type Plotly from 'plotly.js-dist-min'
import { api } from '../api/client'
import type { OptimizeResponse, Options } from '../api/types'
import { ChartLegend } from '../components/ChartLegend'
import { PlotlyChart } from '../components/PlotlyChart'
import { StrategySelect } from '../components/ScenarioForm'
import { HeatScale, ShelfHeatmap } from '../components/ShelfHeatmap'
import { cssVar } from '../lib/robotStates'
import { useThemeVersion } from '../lib/useTheme'

const NAMES: Record<string, string> = {
  random: 'Random (unplanned)',
  closest: 'Closest to stations',
  ga: 'Genetic algorithm',
}
const ORDER = ['random', 'closest', 'ga'] as const

export function OptimizeView({ options }: { options: Options }) {
  const [robots, setRobots] = useState(20)
  const [seed, setSeed] = useState(0)
  const [weight, setWeight] = useState(0.5)
  const [generations, setGenerations] = useState(150)
  const [population, setPopulation] = useState(60)
  const [validate, setValidate] = useState(1500)
  const [planner, setPlanner] = useState('prioritized')
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [result, setResult] = useState<OptimizeResponse | null>(null)
  const theme = useThemeVersion()

  const run = async () => {
    setBusy(true)
    setError(null)
    try {
      setResult(
        await api.optimize({
          config: { num_robots: robots, seed, task_rate: Number((0.015 * robots).toFixed(4)) },
          congestion_weight: weight,
          generations,
          population,
          validate_ticks: validate,
          planner,
        }),
      )
    } catch (e) {
      setError(String(e instanceof Error ? e.message : e))
    } finally {
      setBusy(false)
    }
  }

  const curve = useMemo(() => {
    void theme
    if (!result) return null
    const x = result.history.best.map((_, i) => i + 1)
    const data: Plotly.Data[] = [
      {
        type: 'scatter',
        mode: 'lines',
        x,
        y: result.history.mean,
        name: 'Population mean',
        line: { color: cssVar('--series-2'), width: 2 },
        hovertemplate: '%{y:.2f}<extra>Population mean</extra>',
      },
      {
        type: 'scatter',
        mode: 'lines',
        x,
        y: result.history.best,
        name: 'Best so far',
        line: { color: cssVar('--series-1'), width: 2 },
        hovertemplate: '%{y:.2f}<extra>Best so far</extra>',
      },
    ]
    return data
  }, [result, theme])

  const maxDemand = result ? Math.max(...ORDER.flatMap((k) => result.strategies[k].demand)) : 1
  const best = (key: 'fitness' | 'travel' | 'concentration') =>
    result ? Math.min(...ORDER.map((k) => result.strategies[k][key])) : 0

  return (
    <div className="view">
      <aside className="sidebar panel">
        <h2>Slotting optimiser</h2>
        <p className="sub">
          Decide which shelf holds which product. Hot products near the stations shorten trips; spreading them over
          aisles reduces queues. The GA trades one against the other.
        </p>
        <fieldset className="form" disabled={busy}>
          <label>
            Robots (for the simulation check)
            <input type="number" min={1} max={60} value={robots} onChange={(e) => setRobots(Number(e.target.value))} />
          </label>
          <label>
            Seed
            <input type="number" min={0} value={seed} onChange={(e) => setSeed(Math.max(0, Number(e.target.value)))} />
          </label>
          <label>
            Congestion weight λ
            <input
              type="number"
              min={0}
              max={5}
              step={0.05}
              value={weight}
              onChange={(e) => setWeight(Math.max(0, Number(e.target.value)))}
            />
          </label>
          <label>
            Generations
            <input
              type="number"
              min={10}
              max={2000}
              value={generations}
              onChange={(e) => setGenerations(Number(e.target.value))}
            />
          </label>
          <label>
            Population
            <input type="number" min={4} max={500} value={population} onChange={(e) => setPopulation(Number(e.target.value))} />
          </label>
          <label>
            Simulation check
            <select value={validate} onChange={(e) => setValidate(Number(e.target.value))}>
              <option value={0}>Off (proxy only)</option>
              <option value={1500}>1,500 ticks per strategy</option>
              <option value={3000}>3,000 ticks per strategy</option>
            </select>
          </label>
          <StrategySelect label="Planner for the check" options={options.planners} value={planner} onChange={setPlanner} />
        </fieldset>
        <button className="primary" disabled={busy} onClick={() => void run()}>
          {busy ? 'Optimising…' : 'Run optimiser'}
        </button>
        {error && <p className="error">{error}</p>}
      </aside>

      <main className="main">
        {!result ? (
          <section className="panel empty">
            <h2>SKU slotting</h2>
            <p>{busy ? 'Running the genetic algorithm and simulations…' : 'Set the parameters and run the optimiser.'}</p>
          </section>
        ) : (
          <>
            <section className="panel">
              <h3>Strategies</h3>
              <p className="sub">
                Proxy scores from exact Dijkstra distances ({result.n_aisles} pick aisles, λ = {result.congestion_weight}).
                GA: {result.evaluations.toLocaleString()} evaluations in {result.ga_seconds.toFixed(1)} s.
                {result.validate_ticks > 0 &&
                  ` Simulation: ${result.validate_ticks.toLocaleString()} ticks each, same seed and order stream.`}
              </p>
              <div className="table-wrap">
                <table className="compare">
                  <thead>
                    <tr>
                      <th>Slotting</th>
                      <th className="num">Travel</th>
                      <th className="num">Aisle concentration</th>
                      <th className="num">Fitness</th>
                      {result.validate_ticks > 0 && <th className="num">Sim throughput / 1k</th>}
                      {result.validate_ticks > 0 && <th className="num">Sim avg delivery</th>}
                    </tr>
                  </thead>
                  <tbody>
                    {ORDER.map((k) => {
                      const s = result.strategies[k]
                      const mark = (v: number, key: 'fitness' | 'travel' | 'concentration') =>
                        Math.abs(v - best(key)) < 1e-9 ? 'delta-good' : ''
                      const sims = ORDER.map((n) => result.strategies[n].simulation)
                      const bestThr = Math.max(...sims.map((x) => x?.throughput_per_1k ?? -1))
                      const bestDt = Math.min(...sims.map((x) => x?.delivery_time_mean ?? Infinity))
                      return (
                        <tr key={k}>
                          <td>{NAMES[k]}</td>
                          <td className={`num ${mark(s.travel, 'travel')}`}>{s.travel.toFixed(2)}</td>
                          <td className={`num ${mark(s.concentration, 'concentration')}`}>{s.concentration.toFixed(2)}</td>
                          <td className={`num ${mark(s.fitness, 'fitness')}`}>{s.fitness.toFixed(2)}</td>
                          {result.validate_ticks > 0 && (
                            <td className={`num ${s.simulation?.throughput_per_1k === bestThr ? 'delta-good' : ''}`}>
                              {s.simulation?.throughput_per_1k?.toFixed(1) ?? '–'}
                            </td>
                          )}
                          {result.validate_ticks > 0 && (
                            <td className={`num ${s.simulation?.delivery_time_mean === bestDt ? 'delta-good' : ''}`}>
                              {s.simulation?.delivery_time_mean?.toFixed(1) ?? '–'}
                            </td>
                          )}
                        </tr>
                      )
                    })}
                  </tbody>
                </table>
              </div>
              <p className="sub">
                Travel: demand-weighted distance from pick point to the stations. Aisle concentration: 1 = demand spread
                evenly over all aisles, {result.n_aisles} = all in one aisle. Green = best in column. Lower is better
                everywhere except throughput.
              </p>
            </section>

            <section className="panel">
              <h3>Where the demand sits</h3>
              <p className="sub">Each shelf shaded by its share of orders · outlined squares are packing stations</p>
              <HeatScale />
              <div className="heatmaps">
                {ORDER.map((k) => (
                  <div key={k} className="pair-item">
                    <h3>{NAMES[k]}</h3>
                    <ShelfHeatmap
                      layout={result.layout}
                      shelves={result.shelves}
                      demand={result.strategies[k].demand}
                      max={maxDemand}
                      label={`Demand heatmap, ${NAMES[k]}`}
                    />
                  </div>
                ))}
              </div>
            </section>

            {curve && (
              <section className="panel">
                <h3>GA convergence</h3>
                <p className="sub">Fitness per generation (lower is better)</p>
                <ChartLegend
                  items={[
                    { label: 'Best so far', token: '--series-1' },
                    { label: 'Population mean', token: '--series-2' },
                  ]}
                />
                <PlotlyChart
                  data={curve}
                  layout={{ xaxis: { title: { text: 'Generation' } }, yaxis: { rangemode: 'normal' } }}
                  ariaLabel="GA fitness per generation"
                />
              </section>
            )}
          </>
        )}
      </main>
    </div>
  )
}
