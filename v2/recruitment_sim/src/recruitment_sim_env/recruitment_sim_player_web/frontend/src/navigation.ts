export type WebRole = "red" | "blue" | "referee";
export type Page = "selector" | "red-player" | "blue-player" | "referee";

export function pageForPath(pathname: string): Page {
  const path = pathname.replace(/\/+$/, "") || "/";
  if (path === "/player/red") return "red-player";
  if (path === "/player/blue") return "blue-player";
  if (path === "/referee") return "referee";
  return "selector";
}

export function routeForRole(role: WebRole): string {
  return role === "referee" ? "/referee" : `/player/${role}`;
}
