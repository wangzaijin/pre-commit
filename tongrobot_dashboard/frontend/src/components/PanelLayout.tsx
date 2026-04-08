import GridLayout, { type Layout } from 'react-grid-layout'
import 'react-grid-layout/css/styles.css'
import 'react-resizable/css/styles.css'
import { useRobotStore } from '../stores/robotStore'
import type { PanelItem } from '../types'
import { BaseStatePanel } from './BaseStatePanel'
import { CameraView } from './CameraView'
import { CommandPanel } from './CommandPanel'
import { LaserScanPanel } from './LaserScanPanel'

function PanelContent({ panel }: { panel: PanelItem }) {
  switch (panel.panelType) {
    case 'camera':
      return <CameraView cameraName={panel.sensorName ?? 'main_camera'} />
    case 'base_state':
      return <BaseStatePanel />
    case 'laser_scan':
      return <LaserScanPanel scanName={panel.sensorName ?? 'base_scan'} />
    case 'command':
      return <CommandPanel />
  }
}

const PANEL_TITLES: Record<PanelItem['panelType'], string> = {
  camera:     'Camera',
  base_state: 'Base State',
  laser_scan: 'Laser Scan',
  command:    'Command',
}

export function PanelLayout() {
  const { panels, removePanel, updatePanelLayouts } = useRobotStore()

  const layouts: Layout[] = panels.map((p) => ({
    i: p.id,
    ...p.layout,
    minW: 2,
    minH: 2,
  }))

  function onLayoutChange(next: Layout[]) {
    updatePanelLayouts(
      next.map(({ i, x, y, w, h }) => ({ id: i, layout: { x, y, w, h } }))
    )
  }

  return (
    <GridLayout
      className="layout"
      layout={layouts}
      cols={12}
      rowHeight={60}
      width={window.innerWidth - 32}
      onLayoutChange={onLayoutChange}
      draggableHandle=".panel-drag-handle"
    >
      {panels.map((panel) => (
        <div key={panel.id} style={{ display: 'flex', flexDirection: 'column' }}>
          {/* Title bar */}
          <div
            className="panel-drag-handle"
            style={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              padding: '0.25rem 0.5rem',
              background: 'var(--surface2)',
              borderBottom: '1px solid var(--border)',
              cursor: 'grab',
              userSelect: 'none',
              borderRadius: '4px 4px 0 0',
              flexShrink: 0,
            }}
          >
            <span style={{ fontSize: '0.75rem', color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
              {PANEL_TITLES[panel.panelType]}
              {panel.sensorName ? ` — ${panel.sensorName}` : ''}
            </span>
            <button
              onClick={() => removePanel(panel.id)}
              style={{
                background: 'none', border: 'none', color: 'var(--text-muted)',
                cursor: 'pointer', fontSize: '0.9rem', lineHeight: 1, padding: '0 2px',
              }}
            >
              ×
            </button>
          </div>
          {/* Panel content */}
          <div style={{
            flex: 1, overflow: 'hidden',
            background: 'var(--surface)',
            border: '1px solid var(--border)',
            borderTop: 'none',
            borderRadius: '0 0 4px 4px',
            padding: '0.5rem',
          }}>
            <PanelContent panel={panel} />
          </div>
        </div>
      ))}
    </GridLayout>
  )
}
