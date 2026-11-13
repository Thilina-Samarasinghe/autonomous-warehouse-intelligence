import { STATE_GROUPS } from '../lib/robotStates'

/** Legend: the colour swatch carries identity, the text carries the meaning. */
export function MapLegend() {
  return (
    <ul className="legend" aria-label="Map legend">
      {STATE_GROUPS.map((g) => (
        <li key={g.key}>
          <span className="swatch dot" style={{ background: `var(${g.token})` }} />
          {g.label}
        </li>
      ))}
      <li>
        <span className="swatch dot" style={{ background: 'var(--status-critical)' }} />! Stranded
      </li>
      <li>
        <span className="swatch cargo" /> Cargo aboard
      </li>
      <li>
        <span className="swatch person" /> Person + safety zone
      </li>
      <li>
        <span className="swatch hatch" /> Blocked aisle
      </li>
      <li>
        <span className="swatch shelf" /> Shelf · D station · C charger
      </li>
    </ul>
  )
}
