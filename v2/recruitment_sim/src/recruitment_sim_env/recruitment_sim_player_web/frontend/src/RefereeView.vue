<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from "vue";

import {
  formatRemainingTime,
  refereeActions,
  type RefereeCommand,
  refereeResultText,
  refereeStateText,
} from "./referee";

interface RobotState {
  name: string;
  team: "red" | "blue";
  kind: "infantry" | "sentry";
  max_hp: number;
  current_hp: number;
  shooter_heat: number;
  heat_limit: number;
  alive: boolean;
  available: boolean;
}

interface RefereeStatus {
  type: "status";
  role: "referee";
  request_pending: boolean;
  match: {
    available: boolean;
    state: number;
    result: number;
    remaining_seconds: number;
    red_victory_points: number;
    blue_victory_points: number;
    control_zone_owner: string;
    end_reason: string;
    error_message: string;
  };
  robots: RobotState[];
}

type Connection = "connecting" | "connected" | "occupied" | "disconnected";
const status = ref<RefereeStatus | null>(null);
const connection = ref<Connection>("connecting");
const operationText = ref("");
const localPending = ref(false);
let socket: WebSocket | null = null;
let reconnectTimer = 0;
let requestSerial = 0;
let exitTimer = 0;
let shuttingDown = false;

const connected = computed(
  () => connection.value === "connected" && Boolean(status.value?.match.available),
);
const matchState = computed(() => status.value?.match.state ?? -1);
const requestPending = computed(
  () => localPending.value || Boolean(status.value?.request_pending),
);
const actions = computed(() => refereeActions(matchState.value, connected.value, requestPending.value));
const canReset = computed(() => actions.value.reset);
const canStart = computed(() => actions.value.start);
const canEnd = computed(() => actions.value.end);
const stateText = computed(() => {
  if (connection.value === "occupied") return "裁判端已被占用";
  if (!connected.value) return "裁判节点离线";
  return refereeStateText(matchState.value);
});
const remainingTime = computed(() => {
  if (!connected.value) return "--:--";
  return formatRemainingTime(status.value?.match.remaining_seconds ?? 0);
});
const zoneOwner = computed(() => {
  const owner = status.value?.match.control_zone_owner;
  return owner === "red" ? "红方" : owner === "blue" ? "蓝方" : "无人";
});
const resultText = computed(() => {
  return refereeResultText(
    matchState.value,
    status.value?.match.result ?? 0,
    status.value?.match.error_message ?? "",
  );
});
const orderedRobots = computed(() => {
  const order = { red: 0, blue: 2 };
  return [...(status.value?.robots ?? [])]
    .filter((robot) => ["red", "blue"].includes(robot.team) && ["infantry", "sentry"].includes(robot.kind))
    .sort((left, right) =>
      order[left.team] + (left.kind === "sentry" ? 1 : 0)
      - order[right.team] - (right.kind === "sentry" ? 1 : 0),
    );
});

function hpPercent(robot: RobotState): number {
  return robot.max_hp <= 0 ? 0 : Math.max(0, Math.min(100, 100 * robot.current_hp / robot.max_hp));
}
function heatPercent(robot: RobotState): number {
  return robot.heat_limit <= 0 ? 0 : Math.max(0, Math.min(100, 100 * robot.shooter_heat / robot.heat_limit));
}
function robotLabel(robot: RobotState): string {
  return `${robot.team === "red" ? "红方" : "蓝方"}${robot.kind === "infantry" ? "步兵" : "哨兵"}`;
}

function connect(): void {
  if (shuttingDown) return;
  connection.value = "connecting";
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${window.location.host}/ws/referee`);
  socket.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "ready") {
      connection.value = "connected";
    } else if (message.type === "occupied") {
      connection.value = "occupied";
      operationText.value = "裁判端已被另一个页面占用";
    } else if (message.type === "status") {
      status.value = message as RefereeStatus;
    } else if (message.type === "referee_result") {
      localPending.value = false;
      operationText.value = message.accepted ? message.message : `操作被拒绝：${message.message}`;
    } else if (message.type === "error") {
      localPending.value = false;
      operationText.value = message.message;
    } else if (message.type === "released") {
      finishExit();
    }
  });
  socket.addEventListener("close", () => {
    localPending.value = false;
    if (connection.value !== "occupied") connection.value = "disconnected";
    if (!shuttingDown && connection.value !== "occupied") {
      window.clearTimeout(reconnectTimer);
      reconnectTimer = window.setTimeout(connect, 1000);
    }
  });
  socket.addEventListener("error", () => socket?.close());
}

function sendCommand(command: RefereeCommand): void {
  if (socket?.readyState !== WebSocket.OPEN || requestPending.value) return;
  localPending.value = true;
  operationText.value = command === "reset" ? "正在重置比赛…" : command === "start" ? "正在开始比赛…" : "正在结束比赛…";
  socket.send(JSON.stringify({
    type: "referee_command",
    request_id: ++requestSerial,
    command,
  }));
}

function finishExit(): void {
  window.clearTimeout(exitTimer);
  window.location.assign("/");
}

function exitToSelector(): void {
  if (shuttingDown) return;
  shuttingDown = true;
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

onMounted(connect);
onBeforeUnmount(() => {
  shuttingDown = true;
  window.clearTimeout(reconnectTimer);
  window.clearTimeout(exitTimer);
  socket?.close();
});
</script>

<template>
  <main class="referee-page">
    <header class="referee-header">
      <div>
        <span class="eyebrow">MATCH CONTROL</span>
        <h1>裁判系统</h1>
      </div>
      <button class="referee-exit" type="button" @click="exitToSelector">返回终端选择</button>
      <div class="connection-pill" :class="{ online: connected }">
        <i></i>{{ connected ? "ONLINE" : connection === "occupied" ? "OCCUPIED" : "OFFLINE" }}
      </div>
    </header>

    <section class="referee-overview">
      <article class="phase-panel">
        <span>比赛状态</span>
        <strong>{{ stateText }}</strong>
      </article>
      <article class="referee-clock">
        <span>剩余时间</span>
        <strong>{{ remainingTime }}</strong>
      </article>
      <article class="zone-panel">
        <span>中央区域</span>
        <strong>{{ zoneOwner }}</strong>
      </article>
    </section>

    <section class="versus-score">
      <article class="red-score"><span>RED</span><strong>{{ status?.match.red_victory_points ?? 0 }}</strong><small>剩余得分点</small></article>
      <b>:</b>
      <article class="blue-score"><span>BLUE</span><strong>{{ status?.match.blue_victory_points ?? 0 }}</strong><small>剩余得分点</small></article>
    </section>

    <section class="referee-robots">
      <article v-for="robot in orderedRobots" :key="robot.name" class="referee-robot" :class="robot.team">
        <header><b>{{ robotLabel(robot) }}</b><i :class="{ online: robot.available }">{{ robot.available ? "在线" : "离线" }}</i></header>
        <div class="metric-row"><span>生命值</span><strong>{{ robot.available ? `${robot.current_hp} / ${robot.max_hp}` : "-- / --" }}</strong></div>
        <div class="metric-track"><i :style="{ width: `${hpPercent(robot)}%` }"></i></div>
        <template v-if="robot.kind === 'infantry'">
          <div class="metric-row heat"><span>枪口热量</span><strong>{{ robot.available ? `${Math.round(robot.shooter_heat)} / ${Math.round(robot.heat_limit)}` : "-- / --" }}</strong></div>
          <div class="metric-track heat"><i :style="{ width: `${heatPercent(robot)}%` }"></i></div>
        </template>
      </article>
    </section>

    <section class="referee-actions">
      <p :class="{ error: matchState === 5 || operationText.includes('拒绝') }">{{ resultText || operationText || "等待裁判操作" }}</p>
      <div>
        <button :disabled="!canReset" @click="sendCommand('reset')">重置比赛</button>
        <button :disabled="!canStart" @click="sendCommand('start')">开始比赛</button>
        <button :disabled="!canEnd" @click="sendCommand('end')">结束比赛</button>
      </div>
    </section>
  </main>
</template>
