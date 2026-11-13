import { useCallback, useEffect, useRef, useState } from 'react'
import { wsUrl } from './client'
import type { Frame, Layout, WsCommand } from './types'
import { History, type HistoryPoint } from '../lib/history'

export type Connection = 'idle' | 'connecting' | 'open' | 'closed' | 'error'

export interface LiveSession {
  connection: Connection
  layout: Layout | null
  frame: Frame | null
  /** Sampled per-tick series for the charts (bounded). */
  history: HistoryPoint[]
  send: (cmd: WsCommand) => void
  /** Send a command and resolve with the next frame (used for lockstep stepping). */
  request: (cmd: WsCommand) => Promise<Frame>
}

interface State {
  id: string | null
  connection: Connection
  layout: Layout | null
  frame: Frame | null
  history: HistoryPoint[]
}

const EMPTY: Omit<State, 'id' | 'connection'> = { layout: null, frame: null, history: [] }

/**
 * Connects to `/ws/sessions/{id}` and keeps the latest layout + frame.
 * State is tagged with the session id, so switching sessions never shows stale
 * data, and re-renders are batched to one per animation frame however fast
 * frames arrive.
 */
export function useLiveSession(sessionId: string | null): LiveSession {
  const [state, setState] = useState<State>({ id: null, connection: 'idle', ...EMPTY })
  const wsRef = useRef<WebSocket | null>(null)
  const waiters = useRef<((f: Frame) => void)[]>([])

  useEffect(() => {
    if (!sessionId) return
    const history = new History()
    let pending: Frame | null = null
    let raf: number | null = null
    const ws = new WebSocket(wsUrl(sessionId))
    wsRef.current = ws
    const update = (patch: Partial<State>) =>
      setState((s) => (s.id === sessionId ? { ...s, ...patch } : { id: sessionId, connection: 'connecting', ...EMPTY, ...patch }))

    const flush = () => {
      raf = null
      if (pending) {
        update({ frame: pending, history: history.points() })
        pending = null
      }
    }

    ws.onopen = () => update({ connection: 'open' })
    ws.onerror = () => update({ connection: 'error' })
    ws.onclose = () => update({ connection: 'closed' })
    ws.onmessage = (ev: MessageEvent<string>) => {
      const msg = JSON.parse(ev.data) as { type: string }
      if (msg.type === 'layout') {
        update({ layout: msg as unknown as Layout })
      } else if (msg.type === 'frame') {
        const f = msg as Frame
        history.add(f)
        pending = f
        const queued = waiters.current
        waiters.current = []
        queued.forEach((resolve) => resolve(f))
        if (raf === null) raf = requestAnimationFrame(flush)
      }
    }
    return () => {
      ws.onclose = null
      ws.onmessage = null
      ws.close()
      if (raf !== null) cancelAnimationFrame(raf)
      waiters.current = []
      wsRef.current = null
    }
  }, [sessionId])

  const send = useCallback((cmd: WsCommand) => {
    const ws = wsRef.current
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(cmd))
  }, [])

  const request = useCallback(
    (cmd: WsCommand) =>
      new Promise<Frame>((resolve, reject) => {
        const ws = wsRef.current
        if (!ws || ws.readyState !== WebSocket.OPEN) {
          reject(new Error('not connected'))
          return
        }
        waiters.current.push(resolve)
        ws.send(JSON.stringify(cmd))
      }),
    [],
  )

  const current = state.id === sessionId
  return {
    connection: !sessionId ? 'idle' : current ? state.connection : 'connecting',
    layout: current ? state.layout : null,
    frame: current ? state.frame : null,
    history: current ? state.history : EMPTY.history,
    send,
    request,
  }
}
