<script setup lang="ts">
import PlayerView from "./PlayerView.vue";
import RefereeView from "./RefereeView.vue";
import RoleSelector from "./RoleSelector.vue";
import { pageForPath } from "./navigation";

const page = pageForPath(window.location.pathname);
const playerTeam = page === "red-player" ? "red" : page === "blue-player" ? "blue" : null;
</script>

<template>
  <PlayerView v-if="playerTeam" :team="playerTeam" />
  <RefereeView v-else-if="page === 'referee'" />
  <RoleSelector v-else />
</template>
