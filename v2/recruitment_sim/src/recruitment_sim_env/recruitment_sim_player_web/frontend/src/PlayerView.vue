<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from "vue";

import { InputAccumulator } from "./input";

const props = defineProps<{ team: "red" | "blue" }>();

interface RobotState {
  name: string;
  team: "red" | "blue";
  kind: "infantry" | "sentry";
  namespace: string;
  max_hp: number;
  current_hp: number;
  alive: boolean;
  available: boolean;
}

interface StatusPayload {
  type: "status";
  player_team: "red" | "blue";
  player_robot: string;
  ros_receive_fps: number;
  match: {
    available: boolean;
    state: number;
    remaining_seconds: number;
    red_victory_points: number;
    blue_victory_points: number;
  };
  robots: RobotState[];
}

const arena = ref<HTMLElement | null>(null);
const video = ref<HTMLVideoElement | null>(null);
const status = ref<StatusPayload | null>(null);
const playerTeam = computed(() => props.team);
const playerRobot = ref("");
const connection = ref<"connecting" | "connected" | "occupied" | "disconnected">("connecting");
const webrtcState = ref("new");
const pointerLocked = ref(false);
const videoReady = ref(false);
const diagnosticsVisible = ref(false);
const hitFlash = ref(false);
const now = ref(performance.now());
const lastStatusAt = ref(0);
const lastVideoFrameAt = ref(0);
const displayedFps = ref(0);
const droppedFrames = ref(0);
const websocketRtt = ref(0);

const input = new InputAccumulator();
let socket: WebSocket | null = null;
let peer: RTCPeerConnection | null = null;
let sessionToken = "";
let sequence = 0;
let reconnectTimer = 0;
let sendTimer = 0;
let clockTimer = 0;
let metricsTimer = 0;
let frameCounter = 0;
let hitTimer = 0;
let exitTimer = 0;
let shuttingDown = false;
let videoCallbackStarted = false;

const orderedRobots = (team: "red" | "blue") =>
  computed(() =>
    (status.value?.robots ?? [])
      .filter((robot) => robot.team === team && ["infantry", "sentry"].includes(robot.kind))
      .sort((a, b) => (a.kind === "infantry" ? -1 : b.kind === "infantry" ? 1 : 0)),
  );

const redRobots = orderedRobots("red");
const blueRobots = orderedRobots("blue");
const localRobot = computed(
  () => status.value?.robots.find((robot) => robot.name === playerRobot.value) ?? null,
);
const lowHp = computed(() => {
  const robot = localRobot.value;
  return Boolean(robot && robot.max_hp > 0 && robot.current_hp > 0 && robot.current_hp / robot.max_hp <= 0.2);
});
const dead = computed(() => Boolean(localRobot.value?.available && localRobot.value.current_hp <= 0));
const statusStalled = computed(() => connection.value === "connected" && now.value - lastStatusAt.value > 1500);
const videoStalled = computed(() => videoReady.value && now.value - lastVideoFrameAt.value > 1200);
const canEnter = computed(
  () => connection.value === "connected" && Boolean(status.value?.match.available) &&
    videoReady.value && !statusStalled.value && !videoStalled.value,
);
const entryText = computed(() => {
  if (connection.value === "occupied") return "该阵营选手端已被占用";
  if (connection.value === "connecting") return "正在连接选手端…";
  if (connection.value === "disconnected") return "连接中断，正在重连…";
  if (status.value && !status.value.match.available) return "裁判状态连接中断";
  if (!videoReady.value || videoStalled.value) return "正在等待相机画面…";
  if (statusStalled.value) return "裁判状态连接中断";
  return "点击进入比赛";
});
const showBlockingOverlay = computed(() => !pointerLocked.value || !canEnter.value);
const remainingTime = computed(() => {
  const seconds = Math.max(0, Math.ceil(status.value?.match.remaining_seconds ?? 0));
  return `${String(Math.floor(seconds / 60)).padStart(2, "0")}:${String(seconds % 60).padStart(2, "0")}`;
});
const hpPercent = (robot: RobotState | null) =>
  !robot || robot.max_hp <= 0 ? 0 : Math.max(0, Math.min(100, (robot.current_hp / robot.max_hp) * 100));
const robotLabel = (kind: string) => (kind === "infantry" ? "步兵" : "哨兵");

function sendInput(active: boolean): void {
  const payload = input.snapshot(sequence, active);
  sequence = (sequence + 1) >>> 0;
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(payload));
}

function releaseInput(): void {
  input.reset();
  sendInput(false);
}

function finishExit(): void {
  window.clearTimeout(exitTimer);
  window.location.assign("/");
}

function exitToSelector(): void {
  if (shuttingDown) return;
  shuttingDown = true;
  releaseInput();
  if (document.pointerLockElement) document.exitPointerLock();
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ type: "release" }));
    exitTimer = window.setTimeout(() => {
      socket?.close();
      finishExit();
    }, 1500);
    return;
  }
  socket?.close();
  finishExit();
}

function connectWebSocket(): void {
  if (shuttingDown) return;
  connection.value = "connecting";
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${window.location.host}/ws/${playerTeam.value}`);
  socket.addEventListener("message", async (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "ready") {
      connection.value = "connected";
      sessionToken = message.token;
      playerRobot.value = message.player_robot;
      try {
        await startWebRtc();
      } catch {
        socket?.close();
      }
    } else if (message.type === "occupied") {
      connection.value = "occupied";
    } else if (message.type === "status") {
      status.value = message as StatusPayload;
      lastStatusAt.value = performance.now();
    } else if (message.type === "pong") {
      websocketRtt.value = Math.max(0, performance.now() - Number(message.sent_at));
    } else if (message.type === "released") {
      finishExit();
    }
  });
  socket.addEventListener("close", () => {
    releaseInput();
    peer?.close();
    peer = null;
    videoReady.value = false;
    if (connection.value !== "occupied") connection.value = "disconnected";
    if (!shuttingDown && connection.value !== "occupied") {
      window.clearTimeout(reconnectTimer);
      reconnectTimer = window.setTimeout(connectWebSocket, 1000);
    }
  });
  socket.addEventListener("error", () => socket?.close());
}

async function waitForIceGathering(connection: RTCPeerConnection): Promise<void> {
  if (connection.iceGatheringState === "complete") return;
  await new Promise<void>((resolve) => {
    const listener = () => {
      if (connection.iceGatheringState === "complete") {
        connection.removeEventListener("icegatheringstatechange", listener);
        resolve();
      }
    };
    connection.addEventListener("icegatheringstatechange", listener);
  });
}

async function startWebRtc(): Promise<void> {
  peer?.close();
  peer = new RTCPeerConnection({ iceServers: [] });
  peer.addTransceiver("video", { direction: "recvonly" });
  peer.addEventListener("connectionstatechange", () => {
    webrtcState.value = peer?.connectionState ?? "closed";
  });
  peer.addEventListener("track", (event) => {
    if (event.track.kind !== "video" || !video.value) return;
    video.value.srcObject = new MediaStream([event.track]);
    void video.value.play();
    startVideoFrameCounter();
  });
  const offer = await peer.createOffer();
  await peer.setLocalDescription(offer);
  await waitForIceGathering(peer);
  const response = await fetch("/api/webrtc/offer", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      token: sessionToken,
      role: playerTeam.value,
      sdp: peer.localDescription?.sdp,
      type: peer.localDescription?.type,
    }),
  });
  if (!response.ok) throw new Error(`WebRTC offer failed: ${response.status}`);
  const answer = await response.json();
  await peer.setRemoteDescription(answer);
}

function startVideoFrameCounter(): void {
  if (videoCallbackStarted || !video.value) return;
  videoCallbackStarted = true;
  const element = video.value;
  const callback = () => {
    videoReady.value = true;
    lastVideoFrameAt.value = performance.now();
    frameCounter += 1;
    element.requestVideoFrameCallback(callback);
  };
  element.requestVideoFrameCallback(callback);
}

async function enterGame(): Promise<void> {
  if (!canEnter.value || !arena.value) return;
  try {
    if (!document.fullscreenElement) await arena.value.requestFullscreen();
  } catch {
    // Pointer Lock remains usable when fullscreen is denied.
  }
  try {
    await arena.value.requestPointerLock({ unadjustedMovement: true });
  } catch {
    await arena.value.requestPointerLock();
  }
}

function onPointerLockChange(): void {
  pointerLocked.value = document.pointerLockElement === arena.value;
  if (!pointerLocked.value) releaseInput();
}

function inputActive(): boolean {
  return pointerLocked.value && connection.value === "connected"
    && document.hasFocus() && !document.hidden;
}

function onKey(event: KeyboardEvent, pressed: boolean): void {
  if (event.code === "F3" && pressed && !event.repeat) {
    event.preventDefault();
    diagnosticsVisible.value = !diagnosticsVisible.value;
    return;
  }
  if (["Escape", "F3"].includes(event.code) || !inputActive()) return;
  if (event.cancelable) event.preventDefault();
  if (pressed && event.repeat) return;
  input.setKey(event.code, pressed);
}

function onMouseMove(event: MouseEvent): void {
  if (inputActive()) input.move(event.movementX, event.movementY);
}

function onMouseButton(event: MouseEvent, pressed: boolean): void {
  if (!inputActive() || ![0, 1, 2, 3, 4].includes(event.button)) return;
  event.preventDefault();
  input.setButton(event.button, pressed);
}

function onVisibilityChange(): void {
  if (document.hidden) {
    if (document.pointerLockElement) document.exitPointerLock();
    releaseInput();
  }
}

async function updateWebRtcStats(): Promise<void> {
  if (!peer) return;
  const reports = await peer.getStats();
  reports.forEach((report) => {
    if (report.type === "inbound-rtp" && report.kind === "video") {
      droppedFrames.value = Number(report.framesDropped ?? 0);
    }
  });
}

watch(
  () => localRobot.value?.current_hp,
  (current, previous) => {
    if (current === undefined || previous === undefined || current >= previous) return;
    hitFlash.value = true;
    window.clearTimeout(hitTimer);
    hitTimer = window.setTimeout(() => (hitFlash.value = false), 260);
  },
);

onMounted(() => {
  connectWebSocket();
  document.addEventListener("pointerlockchange", onPointerLockChange);
  document.addEventListener("mousemove", onMouseMove);
  document.addEventListener("keydown", (event) => onKey(event, true));
  document.addEventListener("keyup", (event) => onKey(event, false));
  document.addEventListener("mousedown", (event) => onMouseButton(event, true));
  document.addEventListener("mouseup", (event) => onMouseButton(event, false));
  document.addEventListener("contextmenu", (event) => event.preventDefault());
  window.addEventListener("blur", releaseInput);
  document.addEventListener("visibilitychange", onVisibilityChange);
  sendTimer = window.setInterval(
    () => sendInput(inputActive()),
    1000 / 60,
  );
  clockTimer = window.setInterval(() => (now.value = performance.now()), 250);
  metricsTimer = window.setInterval(() => {
    displayedFps.value = frameCounter;
    frameCounter = 0;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: "ping", sent_at: performance.now() }));
    }
    void updateWebRtcStats();
  }, 1000);
});

onBeforeUnmount(() => {
  shuttingDown = true;
  releaseInput();
  window.clearInterval(sendTimer);
  window.clearInterval(clockTimer);
  window.clearInterval(metricsTimer);
  window.clearTimeout(reconnectTimer);
  window.clearTimeout(hitTimer);
  window.clearTimeout(exitTimer);
  document.removeEventListener("visibilitychange", onVisibilityChange);
  peer?.close();
  socket?.close();
});
</script>

<template>
  <main
    ref="arena"
    class="arena"
    :class="[`team-${playerTeam}`, { 'is-hit': hitFlash, 'is-low': lowHp }]"
    @click="showBlockingOverlay && enterGame()"
  >
    <video ref="video" class="camera" autoplay muted playsinline />

    <button class="player-exit" type="button" @click.stop="exitToSelector">返回终端选择</button>

    <header class="scoreboard">
      <section class="score-wing red-wing">
        <span class="team-name">RED</span>
        <strong>{{ status?.match.red_victory_points ?? 0 }}</strong>
        <span class="score-label">剩余得分点</span>
      </section>
      <section class="match-clock">
        <strong>{{ remainingTime }}</strong>
      </section>
      <section class="score-wing blue-wing">
        <span class="score-label">剩余得分点</span>
        <strong>{{ status?.match.blue_victory_points ?? 0 }}</strong>
        <span class="team-name">BLUE</span>
      </section>

      <div class="roster red-roster">
        <article v-for="robot in redRobots" :key="robot.name" class="robot-card">
          <span>{{ robotLabel(robot.kind) }}</span>
          <strong>{{ robot.available ? `${robot.current_hp}/${robot.max_hp}` : "--/--" }}</strong>
          <i><b :style="{ width: `${hpPercent(robot)}%` }" /></i>
        </article>
      </div>
      <div class="roster blue-roster">
        <article v-for="robot in blueRobots" :key="robot.name" class="robot-card">
          <span>{{ robotLabel(robot.kind) }}</span>
          <strong>{{ robot.available ? `${robot.current_hp}/${robot.max_hp}` : "--/--" }}</strong>
          <i><b :style="{ width: `${hpPercent(robot)}%` }" /></i>
        </article>
      </div>
    </header>

    <div class="crosshair" aria-hidden="true">
      <i class="crosshair-ring" />
      <i class="crosshair-line top" />
      <i class="crosshair-line right" />
      <i class="crosshair-line bottom" />
      <i class="crosshair-line left" />
      <i class="crosshair-dot" />
    </div>

    <section class="local-status">
      <div class="robot-badge">{{ playerTeam === "red" ? "R" : "B" }}</div>
      <div class="local-hp">
        <span>步兵生命值</span>
        <strong>{{ localRobot?.current_hp ?? 0 }} / {{ localRobot?.max_hp ?? 0 }}</strong>
        <i><b :style="{ width: `${hpPercent(localRobot)}%` }" /></i>
      </div>
    </section>

    <div class="damage-overlay" aria-hidden="true" />
    <div v-if="dead" class="dead-overlay">
      <strong>机器人已战亡</strong>
    </div>

    <section v-if="diagnosticsVisible" class="diagnostics" @click.stop>
      <strong>PLAYER LINK</strong>
      <span>ROS RX <b>{{ status?.ros_receive_fps ?? 0 }} FPS</b></span>
      <span>DISPLAY <b>{{ displayedFps }} FPS</b></span>
      <span>DROPPED <b>{{ droppedFrames }}</b></span>
      <span>WS RTT <b>{{ websocketRtt.toFixed(1) }} ms</b></span>
      <span>WEBRTC <b>{{ webrtcState }}</b></span>
      <span>TEAM <b>{{ playerTeam.toUpperCase() }}</b></span>
    </section>

    <button
      v-if="showBlockingOverlay"
      class="entry-overlay"
      :disabled="!canEnter"
      type="button"
      @click.stop="enterGame"
    >
      <span class="entry-mark">{{ playerTeam === "red" ? "R" : "B" }}</span>
      <strong>{{ entryText }}</strong>
      <small v-if="canEnter">进入后按 Esc 释放鼠标</small>
    </button>
  </main>
</template>
