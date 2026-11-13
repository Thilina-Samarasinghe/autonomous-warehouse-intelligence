import { PRETTY, type Scenario } from '../lib/scenario'

interface Props {
  value: Scenario
  onChange: (s: Scenario) => void
  disabled?: boolean
}

export function ScenarioForm({ value, onChange, disabled }: Props) {
  const set = <K extends keyof Scenario>(key: K, v: Scenario[K]) => onChange({ ...value, [key]: v })
  return (
    <fieldset className="form" disabled={disabled}>
      <label>
        Robots
        <input
          type="number"
          min={1}
          max={60}
          value={value.robots}
          onChange={(e) => set('robots', Math.max(1, Math.min(60, Number(e.target.value))))}
        />
      </label>
      <label>
        Seed
        <input type="number" min={0} value={value.seed} onChange={(e) => set('seed', Math.max(0, Number(e.target.value)))} />
      </label>
      <label>
        Orders / tick / robot
        <input
          type="number"
          min={0.001}
          max={0.1}
          step={0.001}
          value={value.ratePerRobot}
          onChange={(e) => set('ratePerRobot', Number(e.target.value))}
        />
      </label>
      <label>
        Items per trip
        <input
          type="number"
          min={1}
          max={4}
          value={value.capacity}
          onChange={(e) => set('capacity', Math.max(1, Math.min(4, Number(e.target.value))))}
        />
      </label>
      <label>
        People in aisles
        <input
          type="number"
          min={0}
          max={20}
          value={value.people}
          onChange={(e) => set('people', Math.max(0, Math.min(20, Number(e.target.value))))}
        />
      </label>
      <label className="check">
        <input type="checkbox" checked={value.battery} onChange={(e) => set('battery', e.target.checked)} />
        Battery management
      </label>
      <label className="check">
        <input type="checkbox" checked={value.blockages} onChange={(e) => set('blockages', e.target.checked)} />
        Random aisle blockages
      </label>
    </fieldset>
  )
}

export function StrategySelect({
  label,
  options,
  value,
  onChange,
  disabled,
}: {
  label: string
  options: string[]
  value: string
  onChange: (v: string) => void
  disabled?: boolean
}) {
  return (
    <label>
      {label}
      <select value={value} onChange={(e) => onChange(e.target.value)} disabled={disabled}>
        {options.map((o) => (
          <option key={o} value={o}>
            {PRETTY[o] ?? o}
          </option>
        ))}
      </select>
    </label>
  )
}
