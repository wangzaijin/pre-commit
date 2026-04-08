import { useRobotStore } from '../stores/robotStore'

function Row({ label, value }: { label: string; value: string }) {
  return (
    <tr>
      <td style={{ color: 'var(--text-muted)', paddingRight: '1rem', paddingBottom: '0.25rem' }}>
        {label}
      </td>
      <td style={{ fontVariantNumeric: 'tabular-nums' }}>{value}</td>
    </tr>
  )
}

function fmt(n: number, d = 3) {
  return n.toFixed(d)
}

export function BaseStatePanel() {
  const base = useRobotStore((s) => s.baseState)

  if (!base) {
    return (
      <div style={{ color: 'var(--text-muted)', fontSize: '0.85rem', paddingTop: '0.5rem' }}>
        Waiting for base state…
      </div>
    )
  }

  return (
    <table style={{ borderCollapse: 'collapse', width: '100%', fontSize: '0.85rem' }}>
      <tbody>
        <tr>
          <td colSpan={2} style={{ color: 'var(--accent)', paddingBottom: '0.4rem', fontSize: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
            Position
          </td>
        </tr>
        <Row label="x (m)"     value={fmt(base.x)} />
        <Row label="y (m)"     value={fmt(base.y)} />
        <Row label="θ (rad)"   value={fmt(base.theta)} />
        <tr><td colSpan={2} style={{ height: '0.6rem' }} /></tr>
        <tr>
          <td colSpan={2} style={{ color: 'var(--accent)', paddingBottom: '0.4rem', fontSize: '0.75rem', textTransform: 'uppercase', letterSpacing: '0.05em' }}>
            Velocity
          </td>
        </tr>
        <Row label="vx (m/s)"     value={fmt(base.vx)} />
        <Row label="vy (m/s)"     value={fmt(base.vy)} />
        <Row label="ω (rad/s)"    value={fmt(base.omega)} />
      </tbody>
    </table>
  )
}
