import { useMemo } from 'react'
import type Plotly from 'plotly.js-dist-min'
import type { Frame } from '../api/types'
import type { HistoryPoint } from '../lib/history'
import { STATE_GROUPS, cssVar } from '../lib/robotStates'
import { useThemeVersion } from '../lib/useTheme'
import { ChartLegend } from './ChartLegend'
import { PlotlyChart } from './PlotlyChart'

function withAlpha(hex: string, alpha: number): string {
  const n = parseInt(hex.replace('#', ''), 16)
  return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${alpha})`
}

function line(x: number[], y: number[], color: string, name: string, fill = false): Plotly.Data {
  return {
    type: 'scatter',
    mode: 'lines',
    x,
    y,
    name,
    line: { color, width: 2, shape: 'linear' },
    fill: fill ? 'tozeroy' : 'none',
    fillcolor: fill ? withAlpha(color, 0.1) : undefined,
    hovertemplate: `%{y:,.0f}<extra>${name}</extra>`,
  }
}

export function LiveCharts({ history, frame }: { history: HistoryPoint[]; frame: Frame | null }) {
  const theme = useThemeVersion()

  const charts = useMemo(() => {
    void theme
    const t = history.map((p) => p.t)
    const blue = cssVar('--series-1')
    const deliveries = [line(t, history.map((p) => p.delivered), blue, 'Delivered', true)]
    const backlog = [line(t, history.map((p) => p.pending), blue, 'Waiting orders', true)]
    // Fleet activity: stacked areas, one per state group (legend + tooltip carry identity).
    const activity: Plotly.Data[] = STATE_GROUPS.map((g) => ({
      type: 'scatter',
      mode: 'lines',
      x: t,
      y: history.map((p) => p.groups[g.key]),
      name: g.label,
      stackgroup: 'fleet',
      line: { color: cssVar('--surface-1'), width: 2 },
      fillcolor: cssVar(g.token),
      hovertemplate: `%{y}<extra>${g.label}</extra>`,
    }))
    return { deliveries, backlog, activity }
  }, [history, theme])

  const battery = useMemo(() => {
    void theme
    if (!frame || frame.robots.every((r) => r.battery === null)) return null
    const blue = cssVar('--series-1')
    const data: Plotly.Data[] = [
      {
        type: 'bar',
        x: frame.robots.map((r) => `R${r.id}`),
        y: frame.robots.map((r) => r.battery ?? 0),
        marker: { color: blue },
        hovertemplate: '%{x}: %{y:.0f}%<extra></extra>',
        name: 'Battery',
      },
    ]
    return data
  }, [frame, theme])

  const xTitle = { title: { text: 'Tick' } }
  return (
    <div className="chart-grid">
      <section className="panel">
        <h3>Deliveries</h3>
        <p className="sub">Cumulative orders delivered</p>
        <PlotlyChart empty={history.length < 2} data={charts.deliveries} layout={{ xaxis: xTitle }} ariaLabel="Cumulative deliveries over time" />
      </section>
      <section className="panel">
        <h3>Order backlog</h3>
        <p className="sub">Orders waiting for a robot</p>
        <PlotlyChart empty={history.length < 2} data={charts.backlog} layout={{ xaxis: xTitle }} ariaLabel="Waiting orders over time" />
      </section>
      <section className="panel">
        <h3>Fleet activity</h3>
        <p className="sub">Robots per state group</p>
        <ChartLegend items={[...STATE_GROUPS].reverse().map((g) => ({ label: g.label, token: g.token }))} />
        <PlotlyChart
          empty={history.length < 2}
          data={charts.activity}
          layout={{ xaxis: xTitle, yaxis: { title: { text: 'Robots' } } }}
          ariaLabel="Robots per state over time"
        />
      </section>
      {battery && (
        <section className="panel">
          <h3>Battery levels</h3>
          <p className="sub">Charge per robot, dotted line = 25% low threshold</p>
          <PlotlyChart
            data={battery}
            layout={{
              hovermode: 'closest',
              bargap: 0.5,
              yaxis: { range: [0, 100], title: { text: '%' } },
              shapes: [
                {
                  type: 'line',
                  xref: 'paper',
                  x0: 0,
                  x1: 1,
                  y0: 25,
                  y1: 25,
                  line: { color: cssVar('--text-muted'), width: 1, dash: 'dot' },
                },
              ],
            }}
            ariaLabel="Battery level per robot"
          />
        </section>
      )}
    </div>
  )
}
