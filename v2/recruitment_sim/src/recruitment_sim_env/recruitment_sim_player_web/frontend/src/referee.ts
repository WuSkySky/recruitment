export type RefereeCommand = "reset" | "start" | "end";

const STATE_NAMES = [
  "训练模式", "正在启动", "比赛进行中", "正在结束",
  "比赛结束", "裁判错误", "准备就绪", "正在重置",
];

export function formatRemainingTime(seconds: number): string {
  if (!Number.isFinite(seconds)) return "--:--";
  const total = Math.max(0, Math.ceil(seconds));
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${String(total % 60).padStart(2, "0")}`;
}

export function refereeStateText(state: number): string {
  return STATE_NAMES[state] ?? "未知状态";
}

export function refereeActions(state: number, connected: boolean, pending: boolean) {
  if (!connected || pending) return { reset: false, start: false, end: false };
  return {
    reset: [0, 4, 5].includes(state),
    start: state === 6,
    end: state === 2,
  };
}

export function refereeResultText(state: number, result: number, error: string): string {
  if (state === 5) return error ? `裁判错误：${error}` : "裁判系统错误";
  return ["", "红方胜利", "蓝方胜利", "比赛平局", "比赛已人工结束"][result] ?? "";
}
