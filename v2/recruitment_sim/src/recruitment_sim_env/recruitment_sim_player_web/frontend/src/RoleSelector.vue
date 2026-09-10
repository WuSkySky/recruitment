<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from "vue";

import { routeForRole, type WebRole } from "./navigation";

interface RoleState { occupied: boolean; available: boolean }
interface RolesPayload {
  roles: Record<WebRole, RoleState>;
  referee_online: boolean;
}

const roles = ref<RolesPayload | null>(null);
const loadFailed = ref(false);
let pollTimer = 0;

async function refreshRoles(): Promise<void> {
  try {
    const response = await fetch("/api/roles", { cache: "no-store" });
    if (!response.ok) throw new Error(String(response.status));
    roles.value = await response.json() as RolesPayload;
    loadFailed.value = false;
  } catch {
    loadFailed.value = true;
  }
}

function roleStatus(role: WebRole): string {
  if (loadFailed.value) return "网关离线";
  const state = roles.value?.roles[role];
  if (!state) return "检测中";
  if (!state.available) return "不可用";
  return state.occupied ? "已占用" : "空闲";
}

function openRole(role: WebRole): void {
  if (roles.value?.roles[role]?.occupied) return;
  window.location.assign(routeForRole(role));
}

onMounted(() => {
  void refreshRoles();
  pollTimer = window.setInterval(() => void refreshRoles(), 1000);
});
onBeforeUnmount(() => window.clearInterval(pollTimer));
</script>

<template>
  <main class="role-selector">
    <header class="selector-heading">
      <span>RECRUITMENT SIMULATION</span>
      <h1>比赛终端</h1>
      <p>选择需要进入的界面。每个角色仅允许一个活动会话。</p>
    </header>

    <section class="role-grid">
      <button class="role-card red-role" :disabled="roles?.roles.red.occupied" @click="openRole('red')">
        <span class="role-code">R</span>
        <span class="role-copy"><b>红方选手端</b><small>RED PLAYER</small></span>
        <i :class="{ occupied: roles?.roles.red.occupied }">{{ roleStatus("red") }}</i>
      </button>
      <button class="role-card blue-role" :disabled="roles?.roles.blue.occupied" @click="openRole('blue')">
        <span class="role-code">B</span>
        <span class="role-copy"><b>蓝方选手端</b><small>BLUE PLAYER</small></span>
        <i :class="{ occupied: roles?.roles.blue.occupied }">{{ roleStatus("blue") }}</i>
      </button>
      <button class="role-card referee-role" :disabled="roles?.roles.referee.occupied" @click="openRole('referee')">
        <span class="role-code">J</span>
        <span class="role-copy"><b>裁判端</b><small>REFEREE CONTROL</small></span>
        <i :class="{ occupied: roles?.roles.referee.occupied }">{{ roleStatus("referee") }}</i>
      </button>
    </section>

    <footer class="selector-footer">
      <span :class="{ offline: !roles?.referee_online }"></span>
      {{ roles?.referee_online ? "裁判系统在线" : "等待裁判系统" }}
    </footer>
  </main>
</template>
