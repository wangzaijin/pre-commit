// TypeScript types that mirror the JSON messages sent over the WebSocket.

export interface BaseStateData {
  x: number
  y: number
  theta: number
  vx: number
  vy: number
  omega: number
  timestamp: number
}

export interface RobotStateMessage {
  type: 'robot_state'
  data: {
    base: BaseStateData | null
    timestamp: number
  }
}

export interface CameraFrameMessage {
  type: 'camera_frame'
  name: string
  /** base64-encoded JPEG */
  data: string
  timestamp: number
}

export interface LaserScanData {
  ranges: number[]
  angle_min: number
  angle_max: number
  angle_increment: number
  timestamp: number
}

export interface LaserScanMessage {
  type: 'laser_scan'
  name: string
  data: LaserScanData
}

export interface DashboardPanelConfig {
  type: 'camera' | 'base_state' | 'laser_scan' | 'command'
  name?: string
}

export interface ConfigMessage {
  type: 'config'
  data: {
    enabled?: boolean
    port?: number
    ws_port?: number
    static_dir?: string
    default_panels?: DashboardPanelConfig[]
    max_linear_vel?: number
    max_angular_vel?: number
  }
}

export type IncomingMessage =
  | RobotStateMessage
  | CameraFrameMessage
  | LaserScanMessage
  | ConfigMessage

// Outgoing message shapes (client → server)
export interface SubscribeMessage {
  type: 'subscribe'
  topic: 'robot_state' | 'camera' | 'laser_scan'
  name?: string
  fps?: number
}

export interface UnsubscribeMessage {
  type: 'unsubscribe'
  topic: string
}

export interface CmdVelMessage {
  type: 'cmd_vel'
  linear: number
  angular: number
}

export interface StopMessage {
  type: 'stop'
}

export interface GetConfigMessage {
  type: 'get_config'
}

export type OutgoingMessage =
  | SubscribeMessage
  | UnsubscribeMessage
  | CmdVelMessage
  | StopMessage
  | GetConfigMessage

// Panel layout (react-grid-layout)
export interface PanelItem {
  id: string
  panelType: 'camera' | 'base_state' | 'laser_scan' | 'command'
  /** For camera and laser_scan panels: which sensor */
  sensorName?: string
  /** react-grid-layout layout entry */
  layout: {
    x: number
    y: number
    w: number
    h: number
  }
}
