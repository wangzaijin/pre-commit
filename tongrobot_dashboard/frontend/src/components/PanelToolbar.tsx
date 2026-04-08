import { useState } from 'react'
import { useRobotStore } from '../stores/robotStore'
import type { PanelItem } from '../types'

const PANEL_OPTIONS: Array<{ type: PanelItem['panelType']; label: string }> = [
  { type: 'camera',     label: 'Camera' },
  { type: 'base_state', label: 'Base State' },
  { type: 'laser_scan', label: 'Laser Scan' },
  { type: 'command',    label: 'Command' },
]

function uid() {
  return Math.random().toString(36).slice(2, 10)
}

export function PanelToolbar() {
  const [open, setOpen]  = useState(false)
  const { addPanel, panels, dashboardConfig } = useRobotStore()

  function add(type: PanelItem['panelType']) {
    // Place new panel below existing ones
    const maxY = panels.reduce((m, p) => Math.max(m, p.layout.y + p.layout.h), 0)
    const defaultSizes: Record<PanelItem['panelType'], { w: number; h: number }> = {
      camera:     { w: 4, h: 5 },
      base_state: { w: 3, h: 5 },
      laser_scan: { w: 4, h: 5 },
      command:    { w: 3, h: 4 },
    }
    const { w, h } = defaultSizes[type]
    addPanel({
      id: uid(),
      panelType: type,
      sensorName: type === 'camera' ? 'main_camera'
                : type === 'laser_scan' ? 'base_scan'
                : undefined,
      layout: { x: 0, y: maxY, w, h },
    })
    setOpen(false)
  }

  // Suppress unused-variable warning
  void dashboardConfig

  return (
    <div style={{ position: 'relative' }}>
      <button
        onClick={() => setOpen((v) => !v)}
        style={{
          padding: '0.3rem 0.8rem',
          background: 'var(--accent)',
          border: 'none',
          borderRadius: 4,
          color: '#000',
          cursor: 'pointer',
          fontSize: '0.8rem',
          fontWeight: 600,
        }}
      >
        + Add Panel
      </button>

      {open && (
        <div style={{
          position: 'absolute',
          top: '100%',
          left: 0,
          marginTop: 4,
          background: 'var(--surface2)',
          border: '1px solid var(--border)',
          borderRadius: 4,
          overflow: 'hidden',
          zIndex: 100,
          minWidth: 140,
        }}>
          {PANEL_OPTIONS.map(({ type, label }) => (
            <button
              key={type}
              onClick={() => add(type)}
              style={{
                display: 'block',
                width: '100%',
                padding: '0.4rem 0.75rem',
                textAlign: 'left',
                background: 'none',
                border: 'none',
                color: 'var(--text)',
                cursor: 'pointer',
                fontSize: '0.85rem',
              }}
              onMouseEnter={(e) => (e.currentTarget.style.background = 'var(--border)')}
              onMouseLeave={(e) => (e.currentTarget.style.background = 'none')}
            >
              {label}
            </button>
          ))}
        </div>
      )}
    </div>
  )
}
