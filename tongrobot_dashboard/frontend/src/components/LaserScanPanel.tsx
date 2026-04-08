import { useEffect, useRef } from 'react'
import { useRobotStore } from '../stores/robotStore'
import { ws } from '../services/websocket'

interface Props {
  scanName?: string
}

const MAX_RANGE = 3.5  // metres — clip for display

export function LaserScanPanel({ scanName = 'base_scan' }: Props) {
  const scan = useRobotStore((s) => s.laserScans[scanName])
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    ws.subscribe('laser_scan', { name: scanName })
    return () => ws.unsubscribe('laser_scan')
  }, [scanName])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || !scan) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const { width: W, height: H } = canvas
    const cx = W / 2
    const cy = H / 2
    const scale = Math.min(W, H) / 2 / MAX_RANGE

    ctx.clearRect(0, 0, W, H)

    // Background
    ctx.fillStyle = '#0e1117'
    ctx.fillRect(0, 0, W, H)

    // Range rings
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'
    ctx.lineWidth = 1
    for (let r = 1; r <= MAX_RANGE; r++) {
      ctx.beginPath()
      ctx.arc(cx, cy, r * scale, 0, Math.PI * 2)
      ctx.stroke()
    }

    // Robot
    ctx.fillStyle = 'var(--accent, #7ec8e3)'
    ctx.beginPath()
    ctx.arc(cx, cy, 5, 0, Math.PI * 2)
    ctx.fill()

    // Scan points
    ctx.fillStyle = 'rgba(126, 200, 227, 0.7)'
    scan.ranges.forEach((r, i) => {
      if (!isFinite(r) || r <= 0 || r > MAX_RANGE) return
      const angle = scan.angle_min + i * scan.angle_increment
      // ROS convention: x forward, y left — in canvas: x right, y down
      const px = cx + Math.cos(angle) * r * scale
      const py = cy - Math.sin(angle) * r * scale
      ctx.fillRect(px - 1, py - 1, 2, 2)
    })
  }, [scan])

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      {!scan && (
        <div style={{
          position: 'absolute', inset: 0,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          color: 'var(--text-muted)', fontSize: '0.85rem',
        }}>
          Waiting for scan — {scanName}
        </div>
      )}
      <canvas
        ref={canvasRef}
        width={300}
        height={300}
        style={{ width: '100%', height: '100%' }}
      />
      {scan && (
        <div style={{
          position: 'absolute', bottom: 4, left: 6,
          fontSize: '0.7rem', color: 'rgba(255,255,255,0.4)',
        }}>
          {scan.ranges.filter(isFinite).length} pts
        </div>
      )}
    </div>
  )
}
