import type { Frame } from '../api/types'
import { STATE_GROUPS, STATE_LABEL, groupOf } from '../lib/robotStates'

const TOKEN = Object.fromEntries(STATE_GROUPS.map((g) => [g.key, g.token]))

/** Table view of the fleet: every value the map shows in colour, as text. */
export function RobotTable({ frame }: { frame: Frame | null }) {
  if (!frame) return null
  const battery = frame.robots.some((r) => r.battery !== null)
  return (
    <div className="table-wrap">
      <table className="robots">
        <thead>
          <tr>
            <th>Robot</th>
            <th>State</th>
            <th>Position</th>
            <th>Goal</th>
            <th className="num">Cargo</th>
            {battery && <th className="num">Battery</th>}
          </tr>
        </thead>
        <tbody>
          {frame.robots.map((r) => (
            <tr key={r.id}>
              <td className="num">{r.id}</td>
              <td>
                <span
                  className="swatch dot"
                  style={{
                    background: r.state === 'stranded' ? 'var(--status-critical)' : `var(${TOKEN[groupOf(r.state)]})`,
                  }}
                />
                {STATE_LABEL[r.state]}
              </td>
              <td className="num">
                {r.pos[0]}, {r.pos[1]}
              </td>
              <td className="num">{r.goal ? `${r.goal[0]}, ${r.goal[1]}` : '–'}</td>
              <td className="num">{r.carrying || '–'}</td>
              {battery && <td className="num">{r.battery === null ? '–' : `${r.battery.toFixed(0)}%`}</td>}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
