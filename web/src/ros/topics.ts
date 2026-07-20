// ROS topic naming. Data topics are namespaced (bringup uses namespace:=fmbot); TF stays
// at root (/tf, /tf_static). Override the namespace with ?ns= for a differently-named robot.
const NS = (new URLSearchParams(location.search).get("ns") ?? "fmbot").replace(/^\/+|\/+$/g, "");

// Build a namespaced topic name from a relative one, e.g. ns("actuator/status") -> /fmbot/actuator/status.
export function ns(rel: string): string {
  return NS ? `/${NS}/${rel}` : `/${rel}`;
}

export const NAMESPACE = NS;
