import { describe, expect, it } from "vitest";

import { pageForPath, routeForRole } from "./navigation";

describe("web role navigation", () => {
  it("maps public routes to pages", () => {
    expect(pageForPath("/")).toBe("selector");
    expect(pageForPath("/player/red")).toBe("red-player");
    expect(pageForPath("/player/blue/")).toBe("blue-player");
    expect(pageForPath("/referee")).toBe("referee");
    expect(pageForPath("/unknown")).toBe("selector");
  });

  it("maps every role to its route", () => {
    expect(routeForRole("red")).toBe("/player/red");
    expect(routeForRole("blue")).toBe("/player/blue");
    expect(routeForRole("referee")).toBe("/referee");
  });
});
