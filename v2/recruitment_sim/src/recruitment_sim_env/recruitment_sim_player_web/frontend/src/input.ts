export interface PlayerInputSnapshot {
  type: "input";
  sequence: number;
  active: boolean;
  mouse_dx: number;
  mouse_dy: number;
  pressed_keys: string[];
}

export class InputAccumulator {
  private dx = 0;
  private dy = 0;
  private keys = new Set<string>();


  move(dx: number, dy: number): void {
    this.dx += dx;
    this.dy += dy;
  }

  setKey(code: string, pressed: boolean): void {
    if (!code || ["Unidentified", "Escape", "F3"].includes(code)) return;
    if (pressed) this.keys.add(code);
    else this.keys.delete(code);
  }

  setButton(button: number, pressed: boolean): void {
    const code = ["MouseLeft", "MouseMiddle", "MouseRight", "MouseBack", "MouseForward"][button];
    if (code) this.setKey(code, pressed);
  }

  snapshot(sequence: number, active: boolean): PlayerInputSnapshot {
    if (!active) this.reset();
    const result: PlayerInputSnapshot = {
      type: "input",
      sequence,
      active,
      mouse_dx: active ? this.dx : 0,
      mouse_dy: active ? this.dy : 0,
      pressed_keys: [...this.keys].sort(),
    };
    this.dx = 0;
    this.dy = 0;
    return result;
  }

  reset(): void {
    this.dx = 0;
    this.dy = 0;
    this.keys.clear();
  }
}
