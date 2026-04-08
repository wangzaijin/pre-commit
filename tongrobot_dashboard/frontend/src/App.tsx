import { useEffect } from 'react'
import { connectStore, useRobotStore } from './stores/robotStore'
import { ws } from './services/websocket'
import { ConnectionStatus } from './components/ConnectionStatus'
import { PanelLayout } from './components/PanelLayout'
import { PanelToolbar } from './components/PanelToolbar'
import type { DashboardPanelConfig } from './types'
import './index.css'

// Derive WebSocket URL from the current page URL so no hardcoding is needed.
const WS_URL = `ws://${window.location.host}/ws`

function App() {
  const { panels, addPanel, dashboardConfig } = useRobotStore()

  useEffect(() => {
    // Connect store to WebSocket (idempotent — safe to call once)
    connectStore(WS_URL)
    // Request the dashboard config from the server
    setTimeout(() => ws.send({ type: 'get_config' }), 500)
    // Subscribe to robot state on every (re)connect
    ws.onConnectionChange((connected) => {
      if (connected) ws.subscribe('robot_state')
    })
  }, [])

  // When config arrives and no panels are saved yet, load defaults
  useEffect(() => {
    if (panels.length > 0) return
    const defaults: DashboardPanelConfig[] = dashboardConfig?.default_panels ?? []
    if (defaults.length === 0) return

    let col = 0
    defaults.forEach((cfg, i) => {
      const defaultSizes: Record<string, { w: number; h: number }> = {
        camera:     { w: 4, h: 5 },
        base_state: { w: 3, h: 5 },
        laser_scan: { w: 4, h: 5 },
        command:    { w: 3, h: 4 },
      }
      const { w, h } = defaultSizes[cfg.type] ?? { w: 3, h: 4 }
      addPanel({
        id: `default-${i}`,
        panelType: cfg.type as 'camera' | 'base_state' | 'laser_scan' | 'command',
        sensorName: cfg.name,
        layout: { x: col % 12, y: 0, w, h },
      })
      col += w
    })
  }, [dashboardConfig])

  return (
    <div className="app">
      {/* Top bar */}
      <header className="topbar">
        <span className="topbar-title">TongRobot Dashboard</span>
        <PanelToolbar />
        <ConnectionStatus />
      </header>

      {/* Panel grid */}
      <main className="main">
        <PanelLayout />
      </main>
    </div>
  )
}

export default App
