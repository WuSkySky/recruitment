import { describe, expect, it } from "vitest";

import { formatRemainingTime, refereeActions, refereeResultText, refereeStateText } from "./referee";

describe("referee view model", () => {
  it("formats time and state text", () => {
    expect(formatRemainingTime(300)).toBe("05:00");
    expect(formatRemainingTime(271.2)).toBe("04:32");
    expect(formatRemainingTime(-1)).toBe("00:00");
    expect(refereeStateText(2)).toBe("比赛进行中");
    expect(refereeStateText(99)).toBe("未知状态");
  });

  it("enables only the safe command", () => {
    expect(refereeActions(0, true, false)).toEqual({ reset: true, start: false, end: false });
    expect(refereeActions(6, true, false)).toEqual({ reset: false, start: true, end: false });
    expect(refereeActions(2, true, false)).toEqual({ reset: false, start: false, end: true });
    expect(refereeActions(2, false, false)).toEqual({ reset: false, start: false, end: false });
    expect(refereeActions(2, true, true)).toEqual({ reset: false, start: false, end: false });
  });

  it("maps results and errors", () => {
    expect(refereeResultText(4, 1, "")).toBe("红方胜利");
    expect(refereeResultText(4, 4, "")).toBe("比赛已人工结束");
    expect(refereeResultText(5, 0, "bad frame")).toBe("裁判错误：bad frame");
  });
});
