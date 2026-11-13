import { useEffect, useRef, useState } from 'react'
import type Plotly from 'plotly.js-dist-min'
import { cssVar } from '../lib/robotStates'
import { useThemeVersion } from '../lib/useTheme'

interface Props {
  data: Plotly.Data[]
  layout?: Partial<Plotly.Layout>
  height?: number
  ariaLabel: string
  /** Show a placeholder instead of empty axes (e.g. before the first tick). */
  empty?: boolean
}

type PlotlyModule = typeof Plotly
let plotlyPromise: Promise<PlotlyModule> | null = null

/** Plotly (~4.8 MB) is code-split and loaded on first use, so the map shows immediately. */
function loadPlotly(): Promise<PlotlyModule> {
  plotlyPromise ??= import('plotly.js-dist-min').then((m) => (m.default ?? m) as PlotlyModule)
  return plotlyPromise
}

/** Theme-aware Plotly wrapper: recessive hairline axes, system font, no mode bar. */
export function PlotlyChart({ data, layout, height = 220, ariaLabel, empty = false }: Props) {
  const ref = useRef<HTMLDivElement>(null)
  const theme = useThemeVersion()
  const [lib, setLib] = useState<PlotlyModule | null>(null)

  useEffect(() => {
    let alive = true
    void loadPlotly().then((m) => alive && setLib(() => m))
    return () => {
      alive = false
    }
  }, [])

  useEffect(() => {
    const el = ref.current
    if (!el || !lib) return
    if (empty) {
      lib.purge(el)
      return
    }
    const muted = cssVar('--text-muted')
    const secondary = cssVar('--text-secondary')
    const grid = cssVar('--grid')
    const axis = cssVar('--axis')
    const surface = cssVar('--surface-1')
    const axisStyle: Partial<Plotly.LayoutAxis> = {
      gridcolor: grid,
      linecolor: axis,
      zerolinecolor: axis,
      tickfont: { color: muted, size: 11 },
      title: { font: { color: secondary, size: 12 } },
      showline: true,
      ticks: '',
    }
    const base: Partial<Plotly.Layout> = {
      height,
      margin: { l: 48, r: 16, t: 8, b: 36 },
      paper_bgcolor: surface,
      plot_bgcolor: surface,
      font: { family: 'system-ui, -apple-system, "Segoe UI", sans-serif', color: secondary, size: 12 },
      hovermode: 'x unified',
      hoverlabel: { bgcolor: surface, bordercolor: axis, font: { color: cssVar('--text-primary') } },
      // Legends are HTML rows above the chart (ChartLegend), never drawn over data.
      showlegend: false,
      ...layout,
      xaxis: { ...axisStyle, ...layout?.xaxis },
      yaxis: { ...axisStyle, rangemode: 'tozero', ...layout?.yaxis },
    }
    void lib.react(el, data, base, { displayModeBar: false, responsive: true })
  }, [lib, data, layout, height, theme, empty])

  useEffect(() => {
    const el = ref.current
    return () => {
      if (el && plotlyPromise) void plotlyPromise.then((m) => m.purge(el))
    }
  }, [])

  return (
    <div ref={ref} className="chart" role="img" aria-label={ariaLabel} style={{ minHeight: height }}>
      {(!lib || empty) && (
        <p className="sub chart-empty">{lib ? 'No data yet: press Run or Step.' : 'Loading chart…'}</p>
      )}
    </div>
  )
}
