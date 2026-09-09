import { describe, expect, it } from "vitest";

import { InputAccumulator } from "./input";

describe("InputAccumulator", () => {
  it("sums mouse movement only within one snapshot", () => {
    const input = new InputAccumulator();
    input.move(4, -2);
    input.move(3, 5);
    expect(input.snapshot(1, true)).toMatchObject({ mouse_dx: 7, mouse_dy: 3 });
    expect(input.snapshot(2, true)).toMatchObject({ mouse_dx: 0, mouse_dy: 0 });
  });

  it("publishes neutral state when inactive or reset", () => {
    const input = new InputAccumulator();
    input.setKey("KeyW", true);
    input.setButton(0, true);
    expect(input.snapshot(1, false)).toMatchObject({ pressed_keys: [] });
    input.reset();
    expect(input.snapshot(2, true)).toMatchObject({ pressed_keys: [] });
  });
});


describe("held keys", () => {
  it("keeps combinations across snapshots and removes released keys", () => {
    const input = new InputAccumulator();
    for (const code of ["KeyR", "ShiftLeft", "ShiftRight", "KeyR"]) input.setKey(code, true);
    for (let button = 0; button < 5; button++) input.setButton(button, true);
    const expected = ["KeyR", "ShiftLeft", "ShiftRight", "MouseLeft", "MouseMiddle", "MouseRight", "MouseBack", "MouseForward"].sort();
    expect(input.snapshot(1, true).pressed_keys).toEqual(expected);
    expect(input.snapshot(2, true).pressed_keys).toEqual(expected);
    input.setKey("KeyR", false);
    input.setButton(0, false);
    expect(input.snapshot(3, true).pressed_keys).toEqual(expected.filter(code => !["KeyR", "MouseLeft"].includes(code)));
    input.snapshot(4, false);
    expect(input.snapshot(5, true).pressed_keys).toEqual([]);
  });

  it("excludes reserved and unidentified keys", () => {
    const input = new InputAccumulator();
    for (const code of ["", "Unidentified", "Escape", "F3"]) input.setKey(code, true);
    expect(input.snapshot(1, true).pressed_keys).toEqual([]);
  });
});
