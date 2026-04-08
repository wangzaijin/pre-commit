import { useEffect } from 'react'
import { useRobotStore } from '../stores/robotStore'
import { ws } from '../services/websocket'

interface Props {
  cameraName: string
  fps?: number
}

export function CameraView({ cameraName, fps = 10 }: Props) {
  const frame = useRobotStore((s) => s.cameraFrames[cameraName])

  useEffect(() => {
    ws.subscribe('camera', { name: cameraName, fps })
    return () => ws.unsubscribe('camera')
  }, [cameraName, fps])

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', background: '#000', borderRadius: 4, overflow: 'hidden' }}>
      {frame ? (
        <img
          src={`data:image/jpeg;base64,${frame}`}
          alt={cameraName}
          style={{ width: '100%', height: '100%', objectFit: 'contain', display: 'block' }}
        />
      ) : (
        <div style={{
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          height: '100%', color: 'var(--text-muted)', fontSize: '0.85rem',
        }}>
          No signal — {cameraName}
        </div>
      )}
      <div style={{
        position: 'absolute', bottom: 4, left: 6,
        fontSize: '0.7rem', color: 'rgba(255,255,255,0.5)',
        pointerEvents: 'none',
      }}>
        {cameraName}
      </div>
    </div>
  )
}
