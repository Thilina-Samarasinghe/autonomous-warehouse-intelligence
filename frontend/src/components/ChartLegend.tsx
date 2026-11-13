/** HTML legend row above a chart: swatch carries identity, text carries meaning. */
export function ChartLegend({ items }: { items: { label: string; token: string }[] }) {
  return (
    <ul className="legend chart-legend">
      {items.map((i) => (
        <li key={i.label}>
          <span className="swatch line" style={{ background: `var(${i.token})` }} />
          {i.label}
        </li>
      ))}
    </ul>
  )
}
