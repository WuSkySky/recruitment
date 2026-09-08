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
    expect(input.snapshot(1, false)).toMatchObject({ key_w: false, left_button: false });
    input.reset();
    expect(input.snapshot(2, true)).toMatchObject({ key_w: false, left_button: false });
  });
});
