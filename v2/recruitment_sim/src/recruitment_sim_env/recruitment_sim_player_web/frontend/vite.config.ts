import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

export default defineConfig({
  plugins: [vue()],
  build: {
    outDir: "../recruitment_sim_player_web/web",
    emptyOutDir: true,
  },
});
