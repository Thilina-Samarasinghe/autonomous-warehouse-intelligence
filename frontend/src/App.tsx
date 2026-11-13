import { useEffect, useState } from 'react'
import { fetchHealth, type HealthResponse } from './api/client'

type State =
  | { kind: 'loading' }
  | { kind: 'ok'; health: HealthResponse }
  | { kind: 'error'; message: string }

export default function App() {
  const [state, setState] = useState<State>({ kind: 'loading' })

  useEffect(() => {
    const ctrl = new AbortController()
    fetchHealth(ctrl.signal)
      .then((health) => setState({ kind: 'ok', health }))
      .catch((err: unknown) => {
        if (!ctrl.signal.aborted) setState({ kind: 'error', message: String(err) })
      })
    return () => ctrl.abort()
  }, [])

  return (
    <main className="shell">
      <h1>Warehouse Fleet Simulator</h1>
      <p className="subtitle">Multi-robot coordination: A*, CBS, Hungarian, OR-Tools, GA</p>
      <section className="card">
        <h2>Backend</h2>
        {state.kind === 'loading' && <p>Connecting…</p>}
        {state.kind === 'ok' && (
          <p className="ok">
            ● Online · API v{state.health.version}
          </p>
        )}
        {state.kind === 'error' && (
          <p className="err">
            ● Offline. Start it with <code>uv run awi-api</code> in <code>backend/</code>
          </p>
        )}
      </section>
    </main>
  )
}
