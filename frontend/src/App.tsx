import { useEffect, useState } from 'react'
import { api } from './api/client'
import type { Options } from './api/types'
import { CompareView } from './views/CompareView'
import { LiveView } from './views/LiveView'
import { OptimizeView } from './views/OptimizeView'

type Tab = 'live' | 'compare' | 'optimize'
type Backend = { kind: 'loading' } | { kind: 'ok'; version: string; options: Options } | { kind: 'error'; message: string }

const TAB_KEY = 'awi.tab'

function savedTab(): Tab {
  try {
    const t = localStorage.getItem(TAB_KEY)
    return t === 'compare' || t === 'optimize' ? t : 'live'
  } catch {
    return 'live'
  }
}

export default function App() {
  const [tab, setTab] = useState<Tab>(savedTab)
  const [backend, setBackend] = useState<Backend>({ kind: 'loading' })

  useEffect(() => {
    const ctrl = new AbortController()
    Promise.all([api.health(ctrl.signal), api.options()])
      .then(([health, options]) => setBackend({ kind: 'ok', version: health.version, options }))
      .catch((err: unknown) => {
        if (!ctrl.signal.aborted) setBackend({ kind: 'error', message: String(err) })
      })
    return () => ctrl.abort()
  }, [])

  const choose = (t: Tab) => {
    setTab(t)
    try {
      localStorage.setItem(TAB_KEY, t)
    } catch {
      /* storage unavailable: the choice just isn't remembered */
    }
  }

  return (
    <div className="app">
      <header className="header">
        <div className="brand">
          <h1>Warehouse Fleet Simulator</h1>
          <span className="sub">Multi-robot coordination · A* · CBS · Hungarian · OR-Tools</span>
        </div>
        <nav className="tabs" aria-label="Views">
          <button className={tab === 'live' ? 'tab active' : 'tab'} onClick={() => choose('live')}>
            Live
          </button>
          <button className={tab === 'compare' ? 'tab active' : 'tab'} onClick={() => choose('compare')}>
            Compare
          </button>
          <button className={tab === 'optimize' ? 'tab active' : 'tab'} onClick={() => choose('optimize')}>
            Optimise
          </button>
        </nav>
        <div className="backend">
          {backend.kind === 'ok' && <span className="ok">● API v{backend.version}</span>}
          {backend.kind === 'loading' && <span className="sub">Connecting…</span>}
          {backend.kind === 'error' && <span className="err">● Backend offline</span>}
        </div>
      </header>

      {backend.kind === 'error' && (
        <section className="panel empty">
          <h2>Backend offline</h2>
          <p>
            Start it with <code>uv run awi-api</code> in <code>backend/</code>, then reload this page.
          </p>
        </section>
      )}
      {backend.kind === 'ok' && tab === 'live' && <LiveView options={backend.options} />}
      {backend.kind === 'ok' && tab === 'compare' && <CompareView options={backend.options} />}
      {backend.kind === 'ok' && tab === 'optimize' && <OptimizeView options={backend.options} />}
    </div>
  )
}
