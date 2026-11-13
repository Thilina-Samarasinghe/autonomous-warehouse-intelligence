import type { RobotStateName } from '../api/types'

/**
 * Robot states are folded into four groups so the map uses only three validated
 * categorical hues plus a neutral (dataviz rule: no more hues than the job needs).
 * Stranded robots use the reserved "critical" status colour, always with a "!" mark.
 */
export type StateGroup = 'free' | 'fetching' | 'carrying' | 'charging'

export const STATE_GROUPS: { key: StateGroup; label: string; token: string }[] = [
  { key: 'fetching', label: 'To pickup / picking', token: '--series-1' },
  { key: 'carrying', label: 'Carrying / dropping', token: '--series-2' },
  { key: 'charging', label: 'Charging', token: '--series-3' },
  { key: 'free', label: 'Idle / parking', token: '--neutral-mark' },
]

export function groupOf(state: RobotStateName): StateGroup {
  switch (state) {
    case 'to_pickup':
    case 'picking':
      return 'fetching'
    case 'to_drop':
    case 'dropping':
      return 'carrying'
    case 'to_charger':
    case 'charging':
      return 'charging'
    default:
      return 'free'
  }
}

export const STATE_LABEL: Record<RobotStateName, string> = {
  idle: 'Idle',
  to_park: 'Parking',
  to_pickup: 'To pickup',
  picking: 'Picking',
  to_drop: 'To drop',
  dropping: 'Dropping',
  to_charger: 'To charger',
  charging: 'Charging',
  stranded: 'Stranded',
}

/** Read a CSS custom property from the document (theme-aware). */
export function cssVar(name: string): string {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim()
}
