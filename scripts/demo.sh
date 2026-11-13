#!/usr/bin/env bash
# One-command demo (macOS / Linux / Git Bash): starts the API and the dashboard.
# Ctrl+C stops both.
#
#   bash scripts/demo.sh
#
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"

command -v uv >/dev/null || { echo "uv not found on PATH: pip install uv" >&2; exit 1; }
command -v npm >/dev/null || { echo "npm not found: install Node.js 20.19+" >&2; exit 1; }

echo "1/3 backend dependencies";   (cd "$root/backend" && uv sync --quiet)
echo "2/3 dashboard dependencies"; (cd "$root/frontend" && { [ -d node_modules ] || npm install --silent; })

echo "3/3 starting API on :8000 and dashboard on :5173"
(cd "$root/backend" && uv run uvicorn awi.api.main:app --port 8000) &
api=$!
(cd "$root/frontend" && npm run dev -- --port 5173 --strictPort) &
web=$!
trap 'kill $api $web 2>/dev/null || true' EXIT INT TERM

for _ in $(seq 1 60); do
  curl -sf http://localhost:5173/api/health >/dev/null && break
  sleep 1
done
echo "Demo running at http://localhost:5173: open the Compare tab and pick a demo scenario."
wait $api
