/**
 * Singleton WebSocket client with automatic reconnection.
 *
 * Usage:
 *   ws.connect('ws://localhost:3000/ws')
 *   ws.subscribe('robot_state')
 *   ws.onMessage('robot_state', (data) => { ... })
 *   const unsub = ws.onMessage('camera_frame', handler)
 *   unsub()  // remove handler
 */

import type { IncomingMessage, OutgoingMessage, SubscribeMessage } from '../types'

type MessageHandler = (msg: IncomingMessage) => void

class WebSocketService {
  private ws: WebSocket | null = null
  private url = ''
  private reconnectDelay = 1000
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  private connected = false
  private intentionallyClosed = false

  // type → set of handlers
  private handlers = new Map<string, Set<MessageHandler>>()
  // active subscriptions to re-send on reconnect
  private activeSubscriptions: SubscribeMessage[] = []
  // connection state listeners
  private connectionListeners = new Set<(connected: boolean) => void>()

  connect(url: string): void {
    this.url = url
    this.intentionallyClosed = false
    this._open()
  }

  disconnect(): void {
    this.intentionallyClosed = true
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    this.ws?.close()
  }

  send(msg: OutgoingMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(msg))
    }
  }

  subscribe(topic: SubscribeMessage['topic'], options?: Partial<SubscribeMessage>): void {
    const msg: SubscribeMessage = { type: 'subscribe', topic, ...options }
    // Store so we can re-send after reconnect
    this.activeSubscriptions = this.activeSubscriptions.filter(
      (s) => !(s.topic === topic && s.name === msg.name)
    )
    this.activeSubscriptions.push(msg)
    this.send(msg)
  }

  unsubscribe(topic: string): void {
    this.activeSubscriptions = this.activeSubscriptions.filter((s) => s.topic !== topic)
    this.send({ type: 'unsubscribe', topic })
  }

  /**
   * Register a handler for messages of a specific type.
   * Returns an unsubscribe function.
   */
  onMessage(type: string, handler: MessageHandler): () => void {
    if (!this.handlers.has(type)) {
      this.handlers.set(type, new Set())
    }
    this.handlers.get(type)!.add(handler)
    return () => this.handlers.get(type)?.delete(handler)
  }

  onConnectionChange(listener: (connected: boolean) => void): () => void {
    this.connectionListeners.add(listener)
    return () => this.connectionListeners.delete(listener)
  }

  get isConnected(): boolean {
    return this.connected
  }

  // ------------------------------------------------------------------
  // Private
  // ------------------------------------------------------------------

  private _open(): void {
    const ws = new WebSocket(this.url)
    this.ws = ws

    ws.onopen = () => {
      this.connected = true
      this.reconnectDelay = 1000
      this._notifyConnection(true)
      // Re-subscribe to all active topics
      for (const sub of this.activeSubscriptions) {
        ws.send(JSON.stringify(sub))
      }
    }

    ws.onmessage = (event: MessageEvent) => {
      let msg: IncomingMessage
      try {
        msg = JSON.parse(event.data as string) as IncomingMessage
      } catch {
        return
      }
      const typeHandlers = this.handlers.get(msg.type)
      if (typeHandlers) {
        for (const h of typeHandlers) {
          h(msg)
        }
      }
    }

    ws.onerror = () => { /* onclose handles reconnect */ }

    ws.onclose = () => {
      this.connected = false
      this._notifyConnection(false)
      if (!this.intentionallyClosed) {
        this.reconnectTimer = setTimeout(() => {
          this.reconnectDelay = Math.min(this.reconnectDelay * 2, 10_000)
          this._open()
        }, this.reconnectDelay)
      }
    }
  }

  private _notifyConnection(state: boolean): void {
    for (const l of this.connectionListeners) l(state)
  }
}

// Singleton
export const ws = new WebSocketService()
