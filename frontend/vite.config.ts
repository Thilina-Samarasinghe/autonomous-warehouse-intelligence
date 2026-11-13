import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  build: {
    // Plotly is one ~4.8 MB library, loaded lazily in its own chunk (see PlotlyChart).
    chunkSizeWarningLimit: 5000,
  },
  server: {
    port: 5173,
    proxy: {
      // FastAPI backend (uv run awi-api)
      '/api': { target: 'http://127.0.0.1:8000', changeOrigin: true },
      '/ws': { target: 'ws://127.0.0.1:8000', ws: true },
    },
  },
})
