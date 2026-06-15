# AutoBee Dashboard (Next.js + Tailwind)

The non-pilot-facing control panel for AutoBee. Type a natural-language command
and watch the mission unfold live: status/phase, telemetry, crowd analytics,
anomaly alerts, the LLM's plan, the annotated camera feed, and the final report.

## How it fits together

```
 Browser ──(HTTP/SSE)──►  Next.js dashboard (this app, on your Mac :3000)
                                  │  fetch /state, EventSource /events, POST /intent
                                  ▼
                          ROS bridge  (autobee_ui/web_ui_node, in container :8080)
                                  │  ROS 2 topics
                                  ▼
                          AutoBee stack (sim or real Tello)
```

The dashboard is a **pure front-end**. All ROS access happens in the Python
bridge node (`workspace/src/autobee_ui/web_ui_node.py`), which the dashboard
talks to over HTTP. Node.js isn't in the ROS container, so you run this app on
your **host** (your Mac) while the bridge runs in the container.

## Prerequisites

- **Node.js 18.18+** (or 20+). On a Mac: `brew install node`, or grab it from
  [nodejs.org](https://nodejs.org).
- The AutoBee stack running with the bridge exposed on port 8080. Start the
  container with `-p 8080:8080` (see the root README's macOS section), then
  inside it run `ros2 launch autobee_core demo.launch.py` — the bridge
  (`web_ui_node`) comes up automatically.

## Run it

```bash
cd dashboard
npm install          # first time only
cp .env.local.example .env.local   # optional — only if the bridge isn't on :8080
npm run dev
```

Open **http://localhost:3000**. The connection dot turns green when it reaches
the bridge.

### Production build

```bash
npm run build
npm start            # serves on :3000
```

## Configuration

| Env var | Default | Meaning |
|---------|---------|---------|
| `NEXT_PUBLIC_BRIDGE_URL` | `http://localhost:8080` | URL of the ROS bridge. |

Set it in `.env.local` if the container maps the bridge to a different port/host.

## Bridge API (for reference)

The dashboard consumes these endpoints (all CORS-enabled):

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/state` | Current snapshot (JSON). |
| GET | `/events` | Server-Sent Events stream of snapshots (live updates). |
| POST | `/intent` | `{"text": "..."}` → published to `/user_intent`. |
| GET | `/camera.mjpg` | Annotated camera MJPEG (only if OpenCV/cv_bridge load). |

## Project structure

```
dashboard/
├── app/
│   ├── layout.tsx        # root layout + global styles
│   ├── page.tsx          # the dashboard (composition + header)
│   └── globals.css       # Tailwind directives
├── components/           # one file per panel
│   ├── Card.tsx          # shared card + row primitives
│   ├── CommandBar.tsx    # command input + quick-suggestion chips
│   ├── MissionStatus.tsx # phase + event badge
│   ├── Telemetry.tsx     # live pose/speed
│   ├── Camera.tsx        # MJPEG feed
│   ├── CrowdAnalytics.tsx
│   ├── Anomalies.tsx
│   ├── MissionLog.tsx
│   ├── Report.tsx
│   └── Plan.tsx
├── lib/
│   ├── types.ts          # BridgeState shape
│   └── useBridge.ts      # SSE + fetch hook
└── tailwind.config.ts
```
