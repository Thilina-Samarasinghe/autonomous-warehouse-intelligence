export interface HealthResponse {
  status: string
  version: string
}

export async function fetchHealth(signal?: AbortSignal): Promise<HealthResponse> {
  const res = await fetch('/api/health', { signal })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  return (await res.json()) as HealthResponse
}
