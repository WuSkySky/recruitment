export interface PlayerInputSnapshot {
  type: "input";
  sequence: number;
  active: boolean;
  mouse_dx: number;
  mouse_dy: number;
  key_w: boolean;
  key_a: boolean;
  key_s: boolean;
  key_d: boolean;
  left_button: boolean;
  right_button: boolean;
}

export class InputAccumulator {
  private dx = 0;
  private dy = 0;
  private keys = new Set<string>();
  private buttons = new Set<number>();

  move(dx: number, dy: number): void {
    this.dx += dx;
    this.dy += dy;
  }

  setKey(code: string, pressed: boolean): void {
    if (pressed) this.keys.add(code);
    else this.keys.delete(code);
  }

  setButton(button: number, pressed: boolean): void {
    if (pressed) this.buttons.add(button);
    else this.buttons.delete(button);
  }

  snapshot(sequence: number, active: boolean): PlayerInputSnapshot {
    const result: PlayerInputSnapshot = {
      type: "input",
      sequence,
      active,
      mouse_dx: active ? this.dx : 0,
      mouse_dy: active ? this.dy : 0,
      key_w: active && this.keys.has("KeyW"),
      key_a: active && this.keys.has("KeyA"),
      key_s: active && this.keys.has("KeyS"),
      key_d: active && this.keys.has("KeyD"),
      left_button: active && this.buttons.has(0),
      right_button: active && this.buttons.has(2),
    };
    this.dx = 0;
    this.dy = 0;
    return result;
  }

  reset(): void {
    this.dx = 0;
    this.dy = 0;
    this.keys.clear();
    this.buttons.clear();
  }
}
