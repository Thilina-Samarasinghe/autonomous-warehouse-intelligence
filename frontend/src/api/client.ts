import type { Cell, OptimizeRequest, OptimizeResponse, Options, SessionRequest, SessionStatus } from './types'

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, {
    ...init,
    headers: { 'Content-Type': 'application/json', ...init?.headers },
  })
  if (!res.ok) {
    let detail = `HTTP ${res.status}`
    try {
      const body = (await res.json()) as { detail?: unknown }
      if (body.detail) detail = typeof body.detail === 'string' ? body.detail : JSON.stringify(body.detail)
    } catch {
      /* no JSON body */
    }
    throw new Error(detail)
  }
  return (res.status === 204 ? undefined : await res.json()) as T
}

export interface HealthResponse {
  status: string
  version: string
}

export const api = {
  health: (signal?: AbortSignal) => request<HealthResponse>('/api/health', { signal }),
  options: () => request<Options>('/api/options'),
  createSession: (body: SessionRequest) =>
    request<SessionStatus>('/api/sessions', { method: 'POST', body: JSON.stringify(body) }),
  deleteSession: (id: string) => request<void>(`/api/sessions/${id}`, { method: 'DELETE' }),
  optimize: (body: OptimizeRequest) =>
    request<OptimizeResponse>('/api/optimize/slotting', { method: 'POST', body: JSON.stringify(body) }),
  addObstacles: (id: string, cells: Cell[], duration: number) =>
    request<{ blocked: Cell[] }>(`/api/sessions/${id}/obstacles`, {
      method: 'POST',
      body: JSON.stringify({ cells, duration }),
    }),
}

/** Back-compat helper used by the header status dot. */
export const fetchHealth = api.health

export function wsUrl(sessionId: string): string {
  const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
  return `${proto}://${window.location.host}/ws/sessions/${sessionId}`
}
