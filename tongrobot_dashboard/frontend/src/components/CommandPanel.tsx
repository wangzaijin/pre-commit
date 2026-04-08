import { useEffect, useRef, useState } from 'react'
import { ws } from '../services/websocket'

const MAX_LINEAR  = 0.26   // m/s  — TurtleBot3 Waffle default
const MAX_ANGULAR = 1.82   // rad/s

export function CommandPanel() {
  const [linear,  setLinear]  = useState(0)
  const [angular, setAngular] = useState(0)
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)

  function sendCurrent() {
    ws.send({ type: 'cmd_vel', linear, angular })
  }

  function startSending() {
    if (intervalRef.current) return
    sendCurrent()
    intervalRef.current = setInterval(sendCurrent, 100)  // 10 Hz
  }

  function stopSending() {
    if (intervalRef.current) {
      clearInterval(intervalRef.current)
      intervalRef.current = null
    }
  }

  function stopRobot() {
    stopSending()
    ws.send({ type: 'stop' })
    setLinear(0)
    setAngular(0)
  }

  // Update the running command when sliders change while held
  useEffect(() => {
    if (intervalRef.current) sendCurrent()
  }, [linear, angular])

  // Send stop on unmount
  useEffect(() => () => { stopSending(); ws.send({ type: 'stop' }) }, [])

  const sliderStyle: React.CSSProperties = {
    width: '100%', accentColor: 'var(--accent)',
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem', fontSize: '0.85rem' }}>
      <label style={{ display: 'flex', flexDirection: 'column', gap: '0.25rem' }}>
        <span style={{ color: 'var(--text-muted)' }}>
          Linear  <strong>{linear.toFixed(2)} m/s</strong>
        </span>
        <input
          type="range"
          min={-MAX_LINEAR}
          max={MAX_LINEAR}
          step={0.01}
          value={linear}
          style={sliderStyle}
          onChange={(e) => setLinear(parseFloat(e.target.value))}
        />
      </label>

      <label style={{ display: 'flex', flexDirection: 'column', gap: '0.25rem' }}>
        <span style={{ color: 'var(--text-muted)' }}>
          Angular  <strong>{angular.toFixed(2)} rad/s</strong>
        </span>
        <input
          type="range"
          min={-MAX_ANGULAR}
          max={MAX_ANGULAR}
          step={0.01}
          value={angular}
          style={sliderStyle}
          onChange={(e) => setAngular(parseFloat(e.target.value))}
        />
      </label>

      <div style={{ display: 'flex', gap: '0.5rem', marginTop: '0.25rem' }}>
        <button
          onMouseDown={startSending}
          onMouseUp={stopSending}
          onMouseLeave={stopSending}
          onTouchStart={startSending}
          onTouchEnd={stopSending}
          style={{ flex: 1, ...btnStyle('var(--accent)') }}
        >
          Hold to Send
        </button>
        <button onClick={stopRobot} style={{ flex: 1, ...btnStyle('var(--red)') }}>
          Stop
        </button>
      </div>
    </div>
  )
}

function btnStyle(color: string): React.CSSProperties {
  return {
    padding: '0.4rem 0.75rem',
    background: 'transparent',
    border: `1px solid ${color}`,
    color,
    borderRadius: 4,
    cursor: 'pointer',
    fontSize: '0.8rem',
  }
}
