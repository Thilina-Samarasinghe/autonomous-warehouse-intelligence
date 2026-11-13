import type { Frame } from '../api/types'

const fmt = (v: number | null | undefined, digits = 0) =>
  v === null || v === undefined ? '–' : v.toLocaleString(undefined, { maximumFractionDigits: digits })
const pct = (v: number | null | undefined) => (v === null || v === undefined ? '–' : `${(v * 100).toFixed(1)}%`)

/** Hero numbers: one figure per tile, label in secondary ink. */
export function KpiTiles({ frame, battery }: { frame: Frame | null; battery: boolean }) {
  const k = frame?.kpis
  const tiles: { label: string; value: string; hint: string; alert?: boolean }[] = [
    { label: 'Throughput', value: fmt(k?.throughput_per_1k, 1), hint: 'deliveries / 1k ticks' },
    { label: 'Avg delivery', value: fmt(k?.delivery_time_mean, 1), hint: 'ticks, order → drop' },
    { label: 'Delivered', value: fmt(frame?.tasks.delivered), hint: `${fmt(frame?.tasks.pending)} waiting` },
    {
      label: 'Collisions avoided',
      value: fmt(k?.conflicts_total),
      hint: 'would-be conflicts (engine-stopped)',
    },
    { label: 'Blocked time', value: pct(k?.blocked_frac), hint: 'of robot-time' },
    { label: 'Idle time', value: pct(k?.idle_frac), hint: 'no task' },
    { label: 'Planner', value: fmt(k?.plan_ms_mean, 1), hint: 'ms per replan' },
  ]
  if (battery) {
    tiles.push({ label: 'Charging', value: pct(k?.charging_frac), hint: 'of robot-time' })
    tiles.push({
      label: 'Stranded',
      value: fmt(k?.stranded_robots),
      hint: 'robots out of energy',
      alert: (k?.stranded_robots ?? 0) > 0,
    })
  }
  return (
    <div className="kpis">
      {tiles.map((t) => (
        <div className="kpi" key={t.label}>
          <div className="kpi-label">{t.label}</div>
          <div className="kpi-value">
            {t.alert && (
              <span className="status-critical" aria-label="critical">
                ⚠{' '}
              </span>
            )}
            {t.value}
          </div>
          <div className="kpi-hint">{t.hint}</div>
        </div>
      ))}
    </div>
  )
}
