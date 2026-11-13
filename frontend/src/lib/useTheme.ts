import { useEffect, useState } from 'react'

/** Increments whenever the colour scheme changes, so canvases and charts repaint. */
export function useThemeVersion(): number {
  const [version, setVersion] = useState(0)
  useEffect(() => {
    const mq = window.matchMedia('(prefers-color-scheme: dark)')
    const bump = () => setVersion((v) => v + 1)
    mq.addEventListener('change', bump)
    const obs = new MutationObserver(bump)
    obs.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] })
    return () => {
      mq.removeEventListener('change', bump)
      obs.disconnect()
    }
  }, [])
  return version
}
