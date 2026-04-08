import { create } from 'zustand'
import { persist } from 'zustand/middleware'
import { ws } from '../services/websocket'
import type {
  BaseStateData,
  CameraFrameMessage,
  ConfigMessage,
  LaserScanData,
  LaserScanMessage,
  PanelItem,
  RobotStateMessage,
} from '../types'

interface RobotStore {
  // Connection
  isConnected: boolean

  // Robot data
  baseState: BaseStateData | null
  cameraFrames: Record<string, string>   // name → base64 JPEG
  laserScans: Record<string, LaserScanData>  // name → scan data
  dashboardConfig: ConfigMessage['data']

  // Panel layout (persisted to localStorage)
  panels: PanelItem[]

  // Actions
  setConnected: (v: boolean) => void
  setBaseState: (b: BaseStateData | null) => void
  setCameraFrame: (name: string, data: string) => void
  setLaserScan: (name: string, data: LaserScanData) => void
  setDashboardConfig: (cfg: ConfigMessage['data']) => void
  addPanel: (panel: PanelItem) => void
  removePanel: (id: string) => void
  updatePanelLayouts: (updates: Array<{ id: string; layout: PanelItem['layout'] }>) => void
}

export const useRobotStore = create<RobotStore>()(
  persist(
    (set) => ({
      isConnected: false,
      baseState: null,
      cameraFrames: {},
      laserScans: {},
      dashboardConfig: {},
      panels: [],

      setConnected:     (v) => set({ isConnected: v }),
      setBaseState:     (b) => set({ baseState: b }),
      setCameraFrame:   (name, data) =>
        set((s) => ({ cameraFrames: { ...s.cameraFrames, [name]: data } })),
      setLaserScan:     (name, data) =>
        set((s) => ({ laserScans: { ...s.laserScans, [name]: data } })),
      setDashboardConfig: (cfg) => set({ dashboardConfig: cfg }),
      addPanel: (panel) =>
        set((s) => ({ panels: [...s.panels, panel] })),
      removePanel: (id) =>
        set((s) => ({ panels: s.panels.filter((p) => p.id !== id) })),
      updatePanelLayouts: (updates) =>
        set((s) => ({
          panels: s.panels.map((p) => {
            const upd = updates.find((u) => u.id === p.id)
            return upd ? { ...p, layout: upd.layout } : p
          }),
        })),
    }),
    {
      name: 'tongrobot-dashboard',
      // Only persist the panel layout, not live data
      partialize: (s) => ({ panels: s.panels }),
    }
  )
)

/** Connect the store to the WebSocket service.  Call once at app startup. */
export function connectStore(wsUrl: string): void {
  // Connection state
  ws.onConnectionChange((connected) => {
    useRobotStore.getState().setConnected(connected)
  })

  // Incoming robot_state
  ws.onMessage('robot_state', (msg) => {
    const m = msg as RobotStateMessage
    useRobotStore.getState().setBaseState(m.data.base)
  })

  // Incoming camera_frame
  ws.onMessage('camera_frame', (msg) => {
    const m = msg as CameraFrameMessage
    useRobotStore.getState().setCameraFrame(m.name, m.data)
  })

  // Incoming laser_scan
  ws.onMessage('laser_scan', (msg) => {
    const m = msg as LaserScanMessage
    useRobotStore.getState().setLaserScan(m.name, m.data)
  })

  // Incoming config
  ws.onMessage('config', (msg) => {
    const m = msg as ConfigMessage
    useRobotStore.getState().setDashboardConfig(m.data)
  })

  ws.connect(wsUrl)
}
