// 2D rigid-transform helpers and a minimal TF-tree resolver (planar projection of /tf onto XY).
export type Tf2D = { x: number; y: number; yaw: number };
export type Edge = { parent: string; x: number; y: number; yaw: number; wall: number };

// Planar yaw (rotation about +z) extracted from a quaternion.
export function yawFromQuat(x: number, y: number, z: number, w: number): number {
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

// Compose a = root_T_mid with b = mid_T_child, yielding root_T_child.
export function compose(a: Tf2D, b: Tf2D): Tf2D {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw);
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: a.yaw + b.yaw };
}

// Resolve root_T_frame by walking child->parent edges up to `root`; null if any link is missing.
export function resolve(tree: Map<string, Edge>, frame: string, root = "odom", maxDepth = 16): Tf2D | null {
  const chain: Edge[] = [];
  let cur = frame;
  for (let i = 0; i < maxDepth && cur !== root; i++) {
    const e = tree.get(cur);
    if (!e) return null;
    chain.push(e);
    cur = e.parent;
  }
  if (cur !== root) return null;
  let acc: Tf2D = { x: 0, y: 0, yaw: 0 };
  for (let i = chain.length - 1; i >= 0; i--) acc = compose(acc, chain[i]);
  return acc;
}
