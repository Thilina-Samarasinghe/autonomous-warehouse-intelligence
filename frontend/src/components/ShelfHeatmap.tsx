import { useEffect, useRef, useState, type MouseEvent } from 'react'
import type { Cell, Layout } from '../api/types'
import { cssVar } from '../lib/robotStates'
import { useThemeVersion } from '../lib/useTheme'

interface Props {
  layout: Layout
  shelves: Cell[]
  demand: number[]
  /** Shared scale maximum so several heatmaps are comparable. */
  max: number
  cell?: number
  label: string
}

function hexToRgb(hex: string): [number, number, number] {
  const n = parseInt(hex.replace('#', ''), 16)
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255]
}

/**
 * Demand per shelf on the real layout. Sequential encoding: one hue (series-1)
 * blended over the surface, so near-zero recedes into the background in both
 * themes. Square-root scale, because Zipf demand is extremely skewed.
 */
export function ShelfHeatmap({ layout, shelves, demand, max, cell = 12, label }: Props) {
  const ref = useRef<HTMLCanvasElement>(null)
  const theme = useThemeVersion()
  const [tip, setTip] = useState<{ x: number; y: number; text: string } | null>(null)
  const index = useRef(new Map<string, number>())

  useEffect(() => {
    index.current = new Map(shelves.map(([r, c], i) => [`${r},${c}`, i]))
  }, [shelves])

  useEffect(() => {
    const canvas = ref.current
    const g = canvas?.getContext('2d')
    if (!canvas || !g) return
    const dpr = window.devicePixelRatio || 1
    canvas.width = layout.width * cell * dpr
    canvas.height = layout.height * cell * dpr
    canvas.style.width = `${layout.width * cell}px`
    canvas.style.height = `${layout.height * cell}px`
    g.setTransform(dpr, 0, 0, dpr, 0, 0)
    const surface = hexToRgb(cssVar('--surface-1'))
    const hue = hexToRgb(cssVar('--series-1'))
    g.fillStyle = cssVar('--map-floor')
    g.fillRect(0, 0, layout.width * cell, layout.height * cell)
    const T = layout.cell_types
    layout.cells.forEach((row, r) =>
      row.forEach((v, c) => {
        if (v === T.pick) {
          g.fillStyle = cssVar('--map-pick')
          g.fillRect(c * cell, r * cell, cell, cell)
        }
      }),
    )
    const scale = Math.sqrt(max) || 1
    shelves.forEach(([r, c], i) => {
      const t = Math.min(1, Math.sqrt(demand[i]) / scale)
      const mix = surface.map((s, k) => Math.round(s + (hue[k] - s) * (0.08 + 0.92 * t)))
      g.fillStyle = `rgb(${mix[0]}, ${mix[1]}, ${mix[2]})`
      g.fillRect(c * cell + 1, r * cell + 1, cell - 2, cell - 2)
    })
    g.strokeStyle = cssVar('--text-secondary')
    g.lineWidth = 1.5
    for (const [r, c] of layout.drop_stations) g.strokeRect(c * cell + 1.5, r * cell + 1.5, cell - 3, cell - 3)
  }, [layout, shelves, demand, max, cell, theme])

  const onMove = (ev: MouseEvent<HTMLCanvasElement>) => {
    const rect = ev.currentTarget.getBoundingClientRect()
    const c = Math.floor((ev.clientX - rect.left) / cell)
    const r = Math.floor((ev.clientY - rect.top) / cell)
    const i = index.current.get(`${r},${c}`)
    if (i === undefined) {
      setTip(null)
      return
    }
    setTip({
      x: ev.clientX - rect.left + 10,
      y: ev.clientY - rect.top + 10,
      text: `Shelf (${r}, ${c}) · ${(demand[i] * 100).toFixed(2)}% of orders`,
    })
  }

  return (
    <div className="canvas-wrap heatmap">
      <canvas ref={ref} onMouseMove={onMove} onMouseLeave={() => setTip(null)} role="img" aria-label={label} />
      {tip && (
        <div className="tooltip" style={{ left: tip.x, top: tip.y }}>
          {tip.text}
        </div>
      )}
    </div>
  )
}

/** Colour scale for the heatmaps (sequential, low → high). */
export function HeatScale() {
  return (
    <div className="heat-scale" aria-hidden="true">
      <span>low demand</span>
      <span className="bar" />
      <span>high (√ scale)</span>
    </div>
  )
}
