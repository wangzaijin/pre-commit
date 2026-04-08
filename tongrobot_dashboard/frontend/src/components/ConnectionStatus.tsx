import { useRobotStore } from '../stores/robotStore'

export function ConnectionStatus() {
  const { isConnected, dashboardConfig } = useRobotStore()

  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
      <span
        style={{
          display: 'inline-block',
          width: 10,
          height: 10,
          borderRadius: '50%',
          background: isConnected ? 'var(--green)' : 'var(--red)',
          boxShadow: isConnected ? '0 0 6px var(--green)' : '0 0 6px var(--red)',
          flexShrink: 0,
        }}
      />
      <span style={{ fontSize: '0.8rem', color: 'var(--text-muted)' }}>
        {isConnected
          ? `Connected${dashboardConfig ? '' : ''}`
          : 'Disconnected — reconnecting…'}
      </span>
    </div>
  )
}
