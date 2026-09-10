#  (c) 2024-2025 zh

"""
手机网页控制模块
"""

import asyncio
import json
import math
import socket
import subprocess
import threading
import time
from typing import Any
from contextlib import suppress
from dataclasses import dataclass
from aiohttp import WSMsgType, web
from src.scripts.rl_data import LOGGER, STATE


#------------ data----------------
@dataclass
class SpeedProfile:
    """速度档位"""
    x: float
    y: float
    yaw: float


#------------ control----------------
class PhoneWebControl:
    """手机网页控制类"""

    LOGGER = "[PHONE]"
    SLOW_SPEED = SpeedProfile(x=1.2, y=1.5, yaw=2.0)
    FAST_SPEED = SpeedProfile(x=2.5, y=1.5, yaw=2.0)

    STICK_DEADZONE = 0.2
    STICK_TIMEOUT = 0.5
    MODEL_MIN = 0
    MODEL_MAX = 7

    def __init__(self, rl_control, host: str = "0.0.0.0", port: int = 8080):
        self.control = rl_control
        self.host = host
        self.port = int(port)
        self.speed = self.SLOW_SPEED
        self.last_stick_time = time.monotonic()
        self.is_running = False
        self._thread = None
        self._loop = None
        self._runner = None
        self._watchdog_task = None
        self._active_ws = None

        self._start()

    def __del__(self):
        self.stop()

    #------------ lifecycle----------------
    def _start(self):
        """启动网页控制服务"""
        self.is_running = True
        self._thread = threading.Thread(target=self._thread_func, daemon=True)
        self._thread.start()

    def stop(self):
        """停止网页控制服务"""
        if not getattr(self, "is_running", False):
            return

        self.is_running = False
        self._zero_command()

        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _thread_func(self):
        """服务线程主函数"""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        try:
            self._loop.run_until_complete(self._run_server())
            self._loop.run_forever()
        except Exception as e:
            print(f"{LOGGER.ERROR}{self.LOGGER} server error: {e}")
        finally:
            self.is_running = False
            self._zero_command()
            self._loop.run_until_complete(self._cleanup())
            self._loop.close()

    async def _run_server(self):
        """配置并启动 HTTP/WebSocket 服务"""
        app = web.Application()
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/ws", self._handle_ws)

        self._runner = web.AppRunner(app)
        await self._runner.setup()

        site = web.TCPSite(self._runner, self.host, self.port)
        await site.start()

        self._watchdog_task = asyncio.create_task(self._watchdog_loop())
        print(f"{self.LOGGER} Web control: http://{self._local_ip()}:{self.port}/")

    async def _cleanup(self):
        """清理 aiohttp runner"""
        if self._active_ws and not self._active_ws.closed:
            await self._active_ws.close(code=1001, message=b"server shutdown")
        self._active_ws = None

        if self._watchdog_task:
            self._watchdog_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._watchdog_task
            self._watchdog_task = None

        if self._runner:
            await self._runner.cleanup()
            self._runner = None

    #------------ handlers----------------
    async def _handle_index(self, request):
        """返回手机控制网页"""
        html = HTML_PAGE.replace("__SPEED_PROFILES__", self._speed_profiles_json())
        html = html.replace("__STICK_DEADZONE__", f"{self.STICK_DEADZONE:.3f}")
        return web.Response(text=html, content_type="text/html")

    async def _handle_ws(self, request):
        """处理 WebSocket 控制连接"""
        ws = web.WebSocketResponse(heartbeat=1.0)
        await ws.prepare(request)

        if self._active_ws and not self._active_ws.closed:
            await self._active_ws.close(code=1000, message=b"new client connected")
        self._active_ws = ws

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self._handle_ws_text(ws, msg.data)
                elif msg.type == WSMsgType.ERROR:
                    print(f"{LOGGER.WARNING}{self.LOGGER} ws error: {ws.exception()}")
        finally:
            if self._active_ws is ws:
                self._active_ws = None
                self._zero_command()

        return ws

    async def _handle_ws_text(self, ws, text: str):
        """处理 WebSocket 文本消息"""
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            print(f"{LOGGER.WARNING}{self.LOGGER} invalid json: {text}")
            return

        if not isinstance(data, dict):
            print(f"{LOGGER.WARNING}{self.LOGGER} invalid message: {data}")
            return

        msg_type = data.get("type")
        if msg_type == "ping":
            await ws.send_json({"type": "pong"})
        elif msg_type == "button":
            self._apply_button(data.get("button"))
        elif msg_type == "stick":
            self._apply_stick(data)
        else:
            print(f"{LOGGER.WARNING}{self.LOGGER} unknown message type: {msg_type}")

    #------------ command----------------
    def _apply_button(self, button: Any):
        """应用按钮命令"""
        if not isinstance(button, str):
            print(f"{LOGGER.WARNING}{self.LOGGER} invalid button: {button}")
            return

        if button == "up":
            self.control.control_state = STATE.STATE_POS_GETUP
        elif button == "down":
            self.control.control_state = STATE.STATE_POS_GETDOWN
        elif button == "stop":
            self._zero_command()
        elif button == "slowspeed":
            self.speed = self.SLOW_SPEED
        elif button == "fastspeed":
            self.speed = self.FAST_SPEED
        elif button.startswith("rl") and button[2:].isdigit():
            model_flag = int(button[2:])
            if self.MODEL_MIN <= model_flag <= self.MODEL_MAX:
                self.control.control_state = STATE.STATE_RL_RUNNING
                self.control.model_flag = model_flag
            else:
                print(f"{LOGGER.WARNING}{self.LOGGER} model out of range: {button}")
        else:
            print(f"{LOGGER.WARNING}{self.LOGGER} unknown button: {button}")

    def _apply_stick(self, data: dict[str, Any]):
        """应用摇杆命令"""
        self.last_stick_time = time.monotonic()
        x = self._axis(data.get("x", 0.0))
        y = self._axis(data.get("y", 0.0))
        yaw = self._axis(data.get("yaw", 0.0))

        self.control.x = x * self.speed.x
        self.control.y = -y * self.speed.y
        self.control.yaw = -yaw * self.speed.yaw

    def _zero_command(self):
        """速度清零"""
        self.control.x = 0.0
        self.control.y = 0.0
        self.control.yaw = 0.0

    async def _watchdog_loop(self):
        """摇杆超时保护"""
        while self.is_running:
            if (
                self._active_ws
                and not self._active_ws.closed
                and time.monotonic() - self.last_stick_time > self.STICK_TIMEOUT
            ):
                self._zero_command()
            await asyncio.sleep(0.05)

    #------------ utils----------------
    def _speed_profiles_json(self) -> str:
        """生成前端速度显示配置"""
        speed_profiles = {
            "slowspeed": {
                "x": self.SLOW_SPEED.x,
                "y": self.SLOW_SPEED.y,
                "yaw": self.SLOW_SPEED.yaw,
            },
            "fastspeed": {
                "x": self.FAST_SPEED.x,
                "y": self.FAST_SPEED.y,
                "yaw": self.FAST_SPEED.yaw,
            },
        }
        return json.dumps(speed_profiles, separators=(",", ":"))

    def _axis(self, value: Any) -> float:
        """归一化轴值限幅"""
        try:
            x = float(value)
        except (TypeError, ValueError):
            return 0.0

        if not math.isfinite(x):
            return 0.0

        x = max(-1.0, min(1.0, x))
        if abs(x) < self.STICK_DEADZONE:
            return 0.0
        return x

    def _local_ip(self) -> str:
        """获取局域网 IP，用于打印手机访问地址"""

        # 优先用外网路由判断当前局域网 IP
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.settimeout(0.5)
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
                if not ip.startswith("127."):
                    return ip
        except OSError:
            pass

        # 外网不可用时，从本机所有 IP 里找
        try:
            ips = subprocess.check_output(
                ["hostname", "-I"],
                text=True,
                timeout=0.5,
            ).strip().split()
        except Exception:
            return "127.0.0.1"

        # 优先返回 Ubuntu 热点常见 IP
        for ip in ips:
            if ip.startswith("10.42."):
                return ip

        # 其次返回普通局域网 IP
        for ip in ips:
            if (
                ip.startswith("192.168.")
                or ip.startswith("10.")
                or ip.startswith(tuple(f"172.{i}." for i in range(16, 32)))
            ):
                return ip

        return "127.0.0.1"


#------------ web page----------------
HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
  <meta name="apple-mobile-web-app-capable" content="yes" />
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent" />
  <title>Robot Phone Control</title>
  <style>
    * {
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
      user-select: none;
    }

    html {
      height: 100%;
      background: #ffffff;
    }

    body {
      margin: 0;
      min-height: 100%;
      background: #ffffff;
      color: #1f2933;
      font-family: Inter, Arial, Helvetica, sans-serif;
      overflow: hidden;
      touch-action: none;
    }

    .app {
      height: 100dvh;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 14px;
      padding: max(12px, env(safe-area-inset-top)) max(12px, env(safe-area-inset-right)) max(12px, env(safe-area-inset-bottom)) max(12px, env(safe-area-inset-left));
      overflow: hidden;
      border: 3px solid #e5e7eb;
      border-radius: 14px;
      box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.8);
      transition: border-color 180ms ease, box-shadow 180ms ease;
    }

    .app.connected {
      border-color: #22c55e;
      box-shadow: 0 0 0 4px rgba(34, 197, 94, 0.12);
    }

    .app.disconnected {
      border-color: #ef4444;
      box-shadow: 0 0 0 4px rgba(239, 68, 68, 0.12);
    }

    .sticks {
      display: grid;
      grid-template-columns: 1fr auto 1fr;
      gap: 20px;
      align-items: center;
      justify-items: center;
      min-height: 0;
    }

    .stick-wrap {
      display: flex;
      align-items: center;
      justify-content: center;
      min-width: 0;
      min-height: 0;
      width: 100%;
    }

    .speed-readout {
      min-width: 132px;
      display: grid;
      gap: 8px;
      padding: 12px;
      border: 1px solid #e1e6eb;
      border-radius: 12px;
      background: #ffffff;
      box-shadow: 0 10px 24px rgba(15, 23, 42, 0.06);
      font-variant-numeric: tabular-nums;
    }

    .speed-row {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      font-size: 13px;
      font-weight: 700;
      color: #64748b;
    }

    .speed-row strong {
      color: #1f2933;
      font-size: 17px;
      min-width: 58px;
      text-align: right;
    }

    .stick {
      width: min(27.6vw, 210px, 46.8vh);
      aspect-ratio: 1 / 1;
      border: 1px solid #d9e0e7;
      border-radius: 50%;
      position: relative;
      background: #f6f8fa;
      box-shadow:
        inset 0 0 0 1px rgba(255, 255, 255, 0.86),
        0 10px 24px rgba(15, 23, 42, 0.08);
      touch-action: none;
    }

    .stick::before,
    .stick::after {
      content: "";
      position: absolute;
      z-index: 0;
      background: #d1d8e0;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      border-radius: 99px;
    }

    .stick::before {
      width: 2px;
      height: 74%;
    }

    .stick::after {
      width: 74%;
      height: 2px;
    }

    .knob {
      width: 34%;
      aspect-ratio: 1 / 1;
      border-radius: 50%;
      position: absolute;
      z-index: 1;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      background: #ffffff;
      border: 1px solid #c8d1da;
      box-shadow:
        0 8px 18px rgba(15, 23, 42, 0.12),
        inset 0 -3px 8px rgba(15, 23, 42, 0.05);
    }

    .buttons {
      display: grid;
      gap: 10px;
      padding: 10px;
      border: 1px solid #e1e6eb;
      border-radius: 12px;
      background: #ffffff;
      box-shadow: 0 10px 24px rgba(15, 23, 42, 0.06);
    }

    .button-row {
      display: grid;
      gap: 9px;
    }

    .button-row.main-row {
      grid-template-columns: repeat(6, minmax(0, 1fr));
    }

    .button-row.rl-row {
      grid-template-columns: repeat(8, minmax(0, 1fr));
    }

    button {
      height: 46px;
      border: 1px solid #d7dee6;
      border-radius: 10px;
      background: #f8fafc;
      color: #1f2933;
      font-size: 14px;
      font-weight: 700;
      letter-spacing: 0;
      touch-action: manipulation;
      box-shadow: 0 6px 14px rgba(15, 23, 42, 0.06);
      transition: transform 100ms ease, background-color 130ms ease, border-color 130ms ease;
    }

    button:active {
      transform: translateY(1px);
      background: #eef2f6;
      border-color: #c5ced8;
    }

    button.active {
      background: #2563eb;
      border-color: #2563eb;
      color: #ffffff;
      box-shadow: 0 8px 18px rgba(37, 99, 235, 0.18);
    }

    button.stop {
      background: #fee2e2;
      border-color: #fecaca;
      color: #991b1b;
    }

    button.mode {
      background: #f1f5f9;
      border-color: #d7dee6;
      color: #334155;
    }

    button.mode.active {
      background: #0f766e;
      border-color: #0f766e;
      color: #ffffff;
    }

    @media (orientation: portrait) {
      .sticks {
        grid-template-columns: 1fr;
        gap: 8px;
      }

      .stick-wrap:first-child {
        order: 1;
      }

      .speed-readout {
        order: 2;
        width: min(52.8vw, 222px, 22.8vh);
      }

      .stick-wrap:last-child {
        order: 3;
      }

      .stick {
        width: min(52.8vw, 222px, 22.8vh);
      }

      .button-row.rl-row {
        grid-template-columns: repeat(4, minmax(0, 1fr));
      }
    }

    @media (max-height: 520px) {
      .app {
        gap: 8px;
      }

      .buttons {
        padding: 8px;
        gap: 7px;
      }

      .button-row {
        gap: 7px;
      }

      button {
        height: 38px;
        font-size: 13px;
      }
    }
  </style>
</head>
<body>
  <main id="app" class="app disconnected">
    <section class="buttons">
      <div class="button-row main-row">
        <button data-button="up" class="mode">UP</button>
        <button data-button="stop" class="stop">STOP</button>
        <button data-button="down" class="mode">DOWN</button>
        <button data-button="slowspeed" class="active">SLOW</button>
        <button data-button="fastspeed">FAST</button>
        <button id="fullscreenButton" type="button">FULLSCREEN</button>
      </div>
      <div class="button-row rl-row">
        <button data-button="rl0" class="mode">RL0</button>
        <button data-button="rl1" class="mode">RL1</button>
        <button data-button="rl2" class="mode">RL2</button>
        <button data-button="rl3" class="mode">RL3</button>
        <button data-button="rl4" class="mode">RL4</button>
        <button data-button="rl5" class="mode">RL5</button>
        <button data-button="rl6" class="mode">RL6</button>
        <button data-button="rl7" class="mode">RL7</button>
      </div>
    </section>

    <section class="sticks">
      <div class="stick-wrap">
        <div id="moveStick" class="stick" data-axis="move">
          <div class="knob"></div>
        </div>
      </div>
      <div class="speed-readout">
        <div class="speed-row"><span>X</span><strong id="speedX">+0.00</strong></div>
        <div class="speed-row"><span>Y</span><strong id="speedY">+0.00</strong></div>
        <div class="speed-row"><span>Yaw</span><strong id="speedYaw">+0.00</strong></div>
      </div>
      <div class="stick-wrap">
        <div id="yawStick" class="stick" data-axis="yaw">
          <div class="knob"></div>
        </div>
      </div>
    </section>
  </main>

  <script>
    const appEl = document.getElementById("app");
    const fullscreenButton = document.getElementById("fullscreenButton");
    const speedXEl = document.getElementById("speedX");
    const speedYEl = document.getElementById("speedY");
    const speedYawEl = document.getElementById("speedYaw");
    const stickState = { x: 0, y: 0, yaw: 0 };
    const speedProfiles = __SPEED_PROFILES__;
    let activeSpeed = speedProfiles.slowspeed;
    let ws = null;
    let lastPongTime = 0;
    let reconnectTimer = null;

    function setConnected(ok) {
      appEl.classList.toggle("connected", ok);
      appEl.classList.toggle("disconnected", !ok);
    }

    function send(data) {
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        return;
      }
      ws.send(JSON.stringify(data));
    }

    function sendStick() {
      send({
        type: "stick",
        x: stickState.x,
        y: stickState.y,
        yaw: stickState.yaw
      });
    }

    function formatSpeed(value) {
      const rounded = Math.round(value * 100) / 100;
      return `${rounded >= 0 ? "+" : ""}${rounded.toFixed(2)}`;
    }

    function updateSpeedReadout() {
      speedXEl.textContent = formatSpeed(stickState.x * activeSpeed.x);
      speedYEl.textContent = formatSpeed(-stickState.y * activeSpeed.y);
      speedYawEl.textContent = formatSpeed(-stickState.yaw * activeSpeed.yaw);
    }

    function connect() {
      if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
        return;
      }

      const proto = location.protocol === "https:" ? "wss:" : "ws:";
      ws = new WebSocket(`${proto}//${location.host}/ws`);

      ws.onopen = () => {
        lastPongTime = performance.now();
        setConnected(true);
        send({ type: "ping" });
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === "pong") {
            lastPongTime = performance.now();
            setConnected(true);
          }
        } catch (_) {
        }
      };

      ws.onclose = () => {
        setConnected(false);
        scheduleReconnect();
      };

      ws.onerror = () => {
        setConnected(false);
        ws.close();
      };
    }

    function scheduleReconnect() {
      if (reconnectTimer) {
        return;
      }
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connect();
      }, 1000);
    }

    function makeStick(el) {
      const knob = el.querySelector(".knob");
      const mode = el.dataset.axis;
      let pointerId = null;

      function update(clientX, clientY) {
        const rect = el.getBoundingClientRect();
        const radius = rect.width / 2;
        const centerX = rect.left + radius;
        const centerY = rect.top + radius;
        let dx = (clientX - centerX) / radius;
        let dy = (clientY - centerY) / radius;
        const len = Math.hypot(dx, dy);
        if (len > 1) {
          dx /= len;
          dy /= len;
        }

        if (mode === "move") {
          stickState.x = roundAxis(-dy);
          stickState.y = roundAxis(dx);
        } else {
          stickState.yaw = roundAxis(dx);
          dy = 0;
        }

        knob.style.transform = `translate(calc(-50% + ${dx * radius * 0.62}px), calc(-50% + ${dy * radius * 0.62}px))`;
        updateSpeedReadout();
        sendStick();
      }

      function reset() {
        if (mode === "move") {
          stickState.x = 0;
          stickState.y = 0;
        } else {
          stickState.yaw = 0;
        }
        knob.style.transform = "translate(-50%, -50%)";
        updateSpeedReadout();
        sendStick();
      }

      el.addEventListener("pointerdown", (event) => {
        pointerId = event.pointerId;
        el.setPointerCapture(pointerId);
        update(event.clientX, event.clientY);
      });

      el.addEventListener("pointermove", (event) => {
        if (event.pointerId === pointerId) {
          update(event.clientX, event.clientY);
        }
      });

      el.addEventListener("pointerup", (event) => {
        if (event.pointerId === pointerId) {
          if (el.hasPointerCapture(pointerId)) {
            el.releasePointerCapture(pointerId);
          }
          pointerId = null;
          reset();
        }
      });

      el.addEventListener("pointercancel", (event) => {
        if (event.pointerId === pointerId) {
          if (el.hasPointerCapture(pointerId)) {
            el.releasePointerCapture(pointerId);
          }
          pointerId = null;
          reset();
        }
      });
    }

    function roundAxis(value) {
      const deadzone = __STICK_DEADZONE__;
      if (Math.abs(value) < deadzone) {
        return 0;
      }
      return Math.round(value * 1000) / 1000;
    }

    fullscreenButton.addEventListener("click", async () => {
      const root = document.documentElement;
      if (root.requestFullscreen) {
        try {
          await root.requestFullscreen();
        } catch (_) {
        }
        return;
      }

      fullscreenButton.textContent = "ADD TO HOME";
      setTimeout(() => {
        fullscreenButton.textContent = "FULLSCREEN";
      }, 1800);
    });

    document.querySelectorAll("[data-button]").forEach((button) => {
      button.addEventListener("click", () => {
        const name = button.dataset.button;
        if (name === "stop") {
          stickState.x = 0;
          stickState.y = 0;
          stickState.yaw = 0;
          document.querySelectorAll(".knob").forEach((knob) => {
            knob.style.transform = "translate(-50%, -50%)";
          });
          updateSpeedReadout();
          send({ type: "button", button: name });
          return;
        }

        if (name === "slowspeed" || name === "fastspeed") {
          activeSpeed = speedProfiles[name];
          updateSpeedReadout();
          document.querySelectorAll("[data-button='slowspeed'], [data-button='fastspeed']").forEach((item) => {
            item.classList.toggle("active", item === button);
          });
        } else {
          document.querySelectorAll("[data-button='up'], [data-button='down'], .rl-row button").forEach((item) => {
            item.classList.toggle("active", item === button);
          });
        }

        send({ type: "button", button: name });
      });
    });

    makeStick(document.getElementById("moveStick"));
    makeStick(document.getElementById("yawStick"));
    updateSpeedReadout();

    const stickTimer = setInterval(sendStick, 50);
    const pingTimer = setInterval(() => send({ type: "ping" }), 300);
    const connectionTimer = setInterval(() => {
      if (!ws || ws.readyState !== WebSocket.OPEN || performance.now() - lastPongTime > 1000) {
        setConnected(false);
      }
    }, 100);

    window.addEventListener("beforeunload", () => {
      clearInterval(stickTimer);
      clearInterval(pingTimer);
      clearInterval(connectionTimer);
      send({ type: "button", button: "stop" });
      if (ws) {
        ws.close();
      }
    });

    connect();
  </script>
</body>
</html>
"""
