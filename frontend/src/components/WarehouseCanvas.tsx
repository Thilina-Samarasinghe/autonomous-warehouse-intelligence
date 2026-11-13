import { useCallback, useEffect, useRef, useState, type MouseEvent } from 'react'
import type { Cell, Frame, Layout, RobotFrame } from '../api/types'
import { STATE_GROUPS, STATE_LABEL, cssVar, groupOf } from '../lib/robotStates'
import { useThemeVersion } from '../lib/useTheme'

interface Props {
  layout: Layout
  frame: Frame | null
  /** Called with the clicked cell (e.g. to close an aisle). */
  onCellClick?: (cell: Cell) => void
  maxCell?: number
  showPaths?: boolean
}

interface Tip {
  x: number
  y: number
  lines: string[]
}

const TRAIL = 6

/**
 * Canvas renderer. The static map is drawn once into an offscreen canvas; each
 * animation frame then composites robots (interpolated between ticks), planned
 * paths, trails, people with their safety zones, and hatched blockages.
 */
export function WarehouseCanvas({ layout, frame, onCellClick, maxCell = 30, showPaths = true }: Props) {
  const wrapRef = useRef<HTMLDivElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const staticRef = useRef<HTMLCanvasElement | null>(null)
  const [cell, setCell] = useState(20)
  const [tip, setTip] = useState<Tip | null>(null)
  const theme = useThemeVersion()

  // Animation state kept in refs: drawing never triggers React renders.
  const prev = useRef<Map<number, Cell>>(new Map())
  const curr = useRef<Map<number, Cell>>(new Map())
  const trails = useRef<Map<number, Cell[]>>(new Map())
  const arrivedAt = useRef(0)
  const interval = useRef(100)
  const lastT = useRef(-1)
  const frameRef = useRef<Frame | null>(frame)

  // Fit the grid to the container width.
  useEffect(() => {
    const el = wrapRef.current
    if (!el) return
    const ro = new ResizeObserver(([entry]) => {
      const w = entry.contentRect.width
      setCell(Math.max(8, Math.min(maxCell, Math.floor(w / layout.width))))
    })
    ro.observe(el)
    return () => ro.disconnect()
  }, [layout.width, maxCell])

  // Static layer: floor, shelves, pick points, stations, chargers.
  useEffect(() => {
    const dpr = window.devicePixelRatio || 1
    const off = document.createElement('canvas')
    off.width = layout.width * cell * dpr
    off.height = layout.height * cell * dpr
    const g = off.getContext('2d')
    if (!g) return
    g.scale(dpr, dpr)
    const T = layout.cell_types
    const colors = {
      floor: cssVar('--map-floor'),
      shelf: cssVar('--map-shelf'),
      pick: cssVar('--map-pick'),
      grid: cssVar('--grid'),
      station: cssVar('--text-secondary'),
      label: cssVar('--text-secondary'),
    }
    g.fillStyle = colors.floor
    g.fillRect(0, 0, layout.width * cell, layout.height * cell)
    layout.cells.forEach((row, r) =>
      row.forEach((v, c) => {
        const x = c * cell
        const y = r * cell
        if (v === T.shelf) {
          g.fillStyle = colors.shelf
          g.fillRect(x + 1, y + 1, cell - 2, cell - 2)
        } else if (v === T.pick) {
          g.fillStyle = colors.pick
          g.fillRect(x, y, cell, cell)
        }
      }),
    )
    g.strokeStyle = colors.grid
    g.lineWidth = 1
    for (let c = 0; c <= layout.width; c++) {
      g.beginPath()
      g.moveTo(c * cell + 0.5, 0)
      g.lineTo(c * cell + 0.5, layout.height * cell)
      g.stroke()
    }
    for (let r = 0; r <= layout.height; r++) {
      g.beginPath()
      g.moveTo(0, r * cell + 0.5)
      g.lineTo(layout.width * cell, r * cell + 0.5)
      g.stroke()
    }
    const glyph = (cells: Cell[], text: string) => {
      g.font = `600 ${Math.max(8, cell * 0.5)}px system-ui, sans-serif`
      g.textAlign = 'center'
      g.textBaseline = 'middle'
      for (const [r, c] of cells) {
        g.strokeStyle = colors.station
        g.lineWidth = 1.5
        g.strokeRect(c * cell + 2.5, r * cell + 2.5, cell - 5, cell - 5)
        g.fillStyle = colors.label
        g.fillText(text, c * cell + cell / 2, r * cell + cell / 2 + 0.5)
      }
    }
    glyph(layout.drop_stations, 'D')
    glyph(layout.chargers, 'C')
    staticRef.current = off
  }, [layout, cell, theme])

  // New frame: shift positions for interpolation, extend trails.
  useEffect(() => {
    frameRef.current = frame
    if (!frame) return
    if (frame.t < lastT.current) {
      trails.current.clear()
      prev.current.clear()
    }
    const now = performance.now()
    if (lastT.current >= 0 && frame.t > lastT.current) {
      interval.current = Math.min(500, Math.max(30, now - arrivedAt.current))
    }
    arrivedAt.current = now
    lastT.current = frame.t
    prev.current = new Map(curr.current)
    curr.current = new Map(frame.robots.map((r) => [r.id, r.pos]))
    for (const r of frame.robots) {
      let trail = trails.current.get(r.id) ?? []
      const last = trail[trail.length - 1]
      const step = last ? Math.abs(last[0] - r.pos[0]) + Math.abs(last[1] - r.pos[1]) : 0
      // A trail only joins neighbouring cells: after a multi-cell jump (fast
      // playback skips frames) it restarts instead of drawing a diagonal.
      if (step > 1) trail = []
      if (!last || step !== 0) trail.push(r.pos)
      trails.current.set(r.id, trail.slice(-TRAIL))
    }
  }, [frame])

  // Render loop.
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const dpr = window.devicePixelRatio || 1
    canvas.width = layout.width * cell * dpr
    canvas.height = layout.height * cell * dpr
    canvas.style.width = `${layout.width * cell}px`
    canvas.style.height = `${layout.height * cell}px`
    const g = canvas.getContext('2d')
    if (!g) return
    const colors = {
      groups: Object.fromEntries(STATE_GROUPS.map((s) => [s.key, cssVar(s.token)])),
      critical: cssVar('--status-critical'),
      surface: cssVar('--surface-1'),
      ink: cssVar('--text-primary'),
      muted: cssVar('--text-muted'),
      zone: cssVar('--map-zone'),
      hatch: cssVar('--map-hatch'),
      onFill: '#ffffff',
    }
    let handle = 0
    const center = (p: Cell): [number, number] => [p[1] * cell + cell / 2, p[0] * cell + cell / 2]

    const draw = () => {
      handle = requestAnimationFrame(draw)
      g.setTransform(dpr, 0, 0, dpr, 0, 0)
      if (staticRef.current) {
        g.drawImage(staticRef.current, 0, 0, layout.width * cell, layout.height * cell)
      }
      const f = frameRef.current
      if (!f) return
      const alpha = Math.min(1, (performance.now() - arrivedAt.current) / interval.current)

      // Blockages: hatched cells (texture, not a hue).
      for (const [r, c] of f.blocked) {
        g.save()
        g.beginPath()
        g.rect(c * cell, r * cell, cell, cell)
        g.clip()
        g.fillStyle = colors.zone
        g.fillRect(c * cell, r * cell, cell, cell)
        g.strokeStyle = colors.hatch
        g.lineWidth = 1.5
        for (let k = -cell; k < cell * 2; k += 5) {
          g.beginPath()
          g.moveTo(c * cell + k, r * cell)
          g.lineTo(c * cell + k - cell, r * cell + cell)
          g.stroke()
        }
        g.restore()
      }
      // People: safety-zone wash + a person marker.
      for (const [r, c] of f.humans) {
        g.fillStyle = colors.zone
        for (const [dr, dc] of [[0, 0], [1, 0], [-1, 0], [0, 1], [0, -1]]) {
          g.fillRect((c + dc) * cell, (r + dr) * cell, cell, cell)
        }
      }
      for (const [r, c] of f.humans) {
        const [x, y] = center([r, c])
        g.fillStyle = colors.ink
        g.beginPath()
        g.arc(x, y - cell * 0.16, cell * 0.13, 0, Math.PI * 2)
        g.fill()
        g.beginPath()
        g.moveTo(x - cell * 0.2, y + cell * 0.3)
        g.lineTo(x, y - cell * 0.02)
        g.lineTo(x + cell * 0.2, y + cell * 0.3)
        g.closePath()
        g.fill()
      }

      const pos = (r: RobotFrame): [number, number] => {
        const a = prev.current.get(r.id)
        const b = r.pos
        if (!a || Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]) !== 1) return center(b)
        const [ax, ay] = center(a)
        const [bx, by] = center(b)
        return [ax + (bx - ax) * alpha, ay + (by - ay) * alpha]
      }

      // Trails (faint) and planned paths (2px line in the robot's colour).
      for (const r of f.robots) {
        const color = r.state === 'stranded' ? colors.critical : colors.groups[groupOf(r.state)]
        const trail = trails.current.get(r.id) ?? []
        if (trail.length > 1) {
          g.globalAlpha = 0.18
          g.strokeStyle = color
          g.lineWidth = 2
          g.lineCap = 'round'
          g.lineJoin = 'round'
          g.beginPath()
          trail.forEach((p, i) => {
            const [x, y] = center(p)
            if (i === 0) g.moveTo(x, y)
            else g.lineTo(x, y)
          })
          g.stroke()
        }
        if (showPaths && r.path.length) {
          g.globalAlpha = 0.55
          g.strokeStyle = color
          g.lineWidth = 2
          g.beginPath()
          const [sx, sy] = pos(r)
          g.moveTo(sx, sy)
          for (const p of r.path) {
            const [x, y] = center(p)
            g.lineTo(x, y)
          }
          g.stroke()
        }
        g.globalAlpha = 1
      }

      // Robots: filled disc, 2px surface ring, cargo mark, battery arc.
      const radius = cell * 0.36
      for (const r of f.robots) {
        const [x, y] = pos(r)
        const stranded = r.state === 'stranded'
        g.fillStyle = stranded ? colors.critical : colors.groups[groupOf(r.state)]
        g.strokeStyle = colors.surface
        g.lineWidth = 2
        g.beginPath()
        g.arc(x, y, radius, 0, Math.PI * 2)
        g.fill()
        g.stroke()
        if (r.carrying > 0) {
          const s = radius * 0.7
          g.fillStyle = colors.onFill
          g.fillRect(x - s / 2, y - s / 2, s, s)
        }
        if (stranded) {
          g.fillStyle = colors.onFill
          g.font = `700 ${Math.max(8, cell * 0.45)}px system-ui, sans-serif`
          g.textAlign = 'center'
          g.textBaseline = 'middle'
          g.fillText('!', x, y + 0.5)
        }
        if (r.battery !== null && cell >= 14) {
          g.strokeStyle = colors.muted
          g.lineWidth = 2
          g.beginPath()
          g.arc(x, y, radius + 3, -Math.PI / 2, -Math.PI / 2 + (Math.PI * 2 * r.battery) / 100)
          g.stroke()
        }
      }
    }
    draw()
    return () => cancelAnimationFrame(handle)
  }, [layout, cell, theme, showPaths])

  const cellAt = useCallback(
    (ev: MouseEvent<HTMLCanvasElement>): Cell | null => {
      const rect = ev.currentTarget.getBoundingClientRect()
      const c = Math.floor((ev.clientX - rect.left) / cell)
      const r = Math.floor((ev.clientY - rect.top) / cell)
      if (r < 0 || c < 0 || r >= layout.height || c >= layout.width) return null
      return [r, c]
    },
    [cell, layout],
  )

  const onMove = (ev: MouseEvent<HTMLCanvasElement>) => {
    const at = cellAt(ev)
    const f = frameRef.current
    if (!at || !f) {
      setTip(null)
      return
    }
    const [r, c] = at
    const lines: string[] = []
    const robot = f.robots.find((b) => b.pos[0] === r && b.pos[1] === c)
    if (robot) {
      lines.push(`Robot ${robot.id} · ${STATE_LABEL[robot.state]}`)
      if (robot.goal) lines.push(`Goal (${robot.goal[0]}, ${robot.goal[1]})`)
      if (robot.carrying) lines.push(`Carrying ${robot.carrying}`)
      if (robot.battery !== null) lines.push(`Battery ${robot.battery.toFixed(0)}%`)
    } else if (f.humans.some(([hr, hc]) => hr === r && hc === c)) {
      lines.push('Person')
    } else if (f.blocked.some(([br, bc]) => br === r && bc === c)) {
      lines.push('Blocked aisle')
    } else {
      const v = layout.cells[r][c]
      const name = Object.entries(layout.cell_types).find(([, n]) => n === v)?.[0] ?? 'floor'
      lines.push(`${name[0].toUpperCase()}${name.slice(1)} (${r}, ${c})`)
    }
    const rect = ev.currentTarget.getBoundingClientRect()
    setTip({ x: ev.clientX - rect.left + 12, y: ev.clientY - rect.top + 12, lines })
  }

  return (
    <div ref={wrapRef} className="canvas-wrap">
      <canvas
        ref={canvasRef}
        className={onCellClick ? 'map clickable' : 'map'}
        onMouseMove={onMove}
        onMouseLeave={() => setTip(null)}
        onClick={(ev) => {
          const at = cellAt(ev)
          if (at && onCellClick) onCellClick(at)
        }}
        role="img"
        aria-label="Live warehouse map"
      />
      {tip && (
        <div className="tooltip" style={{ left: tip.x, top: tip.y }}>
          {tip.lines.map((l) => (
            <div key={l}>{l}</div>
          ))}
        </div>
      )}
    </div>
  )
}
