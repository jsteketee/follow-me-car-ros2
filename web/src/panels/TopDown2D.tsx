// Top-down 2D view centered on the car: base_link (car, with steered front wheels),
// tag_est_link (fused tag est), nav_goal (committed follow goal), and uwb_link (panned UWB
// anchor), all resolved into odom from /tf. The car stays pinned at canvas center; odom
// orientation is fixed (north up). Auto-scales to keep the tag in frame.
import { useEffect, useRef } from "react";
import { useLive } from "../ros/live";
import { Edge, Tf2D, resolve } from "../ros/tf2d";

const CAR_L = 0.30, CAR_W = 0.18;            // chassis footprint (m) — placeholder box
const TRAIL_MAX = 800, TRAIL_MIN_STEP = 0.02;
const TAG_FRAME = "tag_est_link";            // EKF estimate (parented to base_link)
const TAG_STALE_S = 2.0;
const GOAL_FRAME = "nav_goal";               // committed follow goal from nav_controller (odom)
const GOAL_STALE_S = 2.0;
const STEER_VIS_DEG = 28;                    // visual front-wheel deflection at |steering|=1
const SCALE_MIN = 40, SCALE_MAX = 300;       // px per metre clamp
const EDGE_PAD = 28;                         // px kept between the tag and the frame edge
const TAG_MARGIN = 0.5;                       // m of slack kept past the tag when zooming out
const DEFAULT_SCALE = 120;                    // px/m starting zoom

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

type ToScreen = (wx: number, wy: number) => [number, number];

export function TopDown2D() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { treeRef, statusRef } = useLive();
  const trailRef = useRef<{ x: number; y: number }[]>([]);
  // Effective zoom (px/m). Scroll sets it either way; a fresh tag can only ratchet it DOWN
  // (zoom out) to stay in frame — never back up, so the view stops resizing as the tag moves.
  const scaleRef = useRef(DEFAULT_SCALE);

  useEffect(() => {
    const canvas = canvasRef.current!;
    const ctx = canvas.getContext("2d")!;
    let raf = 0;

    // Scroll to zoom: adjust the user's preferred scale; the render clamp keeps the tag in view.
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      scaleRef.current = clamp(scaleRef.current * Math.exp(-e.deltaY * 0.0015), SCALE_MIN, SCALE_MAX);
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });

    const render = () => {
      raf = requestAnimationFrame(render);
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth, h = canvas.clientHeight;
      if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      const tree = treeRef.current;
      const car = resolve(tree, "base_link");
      const tag = resolve(tree, TAG_FRAME);
      const tagEdge = tree.get(TAG_FRAME);
      const uwb = resolve(tree, "uwb_link");
      const nowS = performance.now() / 1000;
      const tagFresh = !!(tag && tagEdge && nowS - tagEdge.wall < TAG_STALE_S);
      const goal = resolve(tree, GOAL_FRAME);
      const goalEdge = tree.get(GOAL_FRAME);
      const goalFresh = !!(goal && goalEdge && nowS - goalEdge.wall < GOAL_STALE_S);

      if (!car) { banner(ctx, w, "waiting for base_link …"); return; }

      // Trail of car positions.
      const tr = trailRef.current, last = tr[tr.length - 1];
      if (!last || Math.hypot(car.x - last.x, car.y - last.y) > TRAIL_MIN_STEP) {
        tr.push({ x: car.x, y: car.y });
        if (tr.length > TRAIL_MAX) tr.shift();
      }

      // View: car pinned at center. Scroll sets the zoom; a fresh tag can only ratchet it
      // DOWN (zoom out) when it would leave the frame — never back in as the tag nears, so
      // the canvas stops constantly resizing when the tag just moves around.
      let scale = clamp(scaleRef.current, SCALE_MIN, SCALE_MAX);
      if (tagFresh && tag) {
        const reach = Math.hypot(tag.x - car.x, tag.y - car.y) + TAG_MARGIN;
        const fitScale = clamp((Math.min(w, h) / 2 - EDGE_PAD) / Math.max(reach, 0.5), SCALE_MIN, SCALE_MAX);
        if (scale > fitScale) scale = fitScale;   // zoom out only
      }
      scaleRef.current = scale;
      const toS: ToScreen = (wx, wy) => [w / 2 + (wx - car.x) * scale, h / 2 - (wy - car.y) * scale];

      drawGrid(ctx, w, h, car.x, car.y, scale);

      if (tr.length > 1) {
        ctx.strokeStyle = "rgba(120,170,255,0.5)"; ctx.lineWidth = 2; ctx.beginPath();
        tr.forEach((p, i) => { const [sx, sy] = toS(p.x, p.y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
        ctx.stroke();
      }

      if (tag && tagFresh) {
        const [csx, csy] = toS(car.x, car.y), [tsx, tsy] = toS(tag.x, tag.y);
        ctx.strokeStyle = "rgba(240,210,60,0.85)"; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(csx, csy); ctx.lineTo(tsx, tsy); ctx.stroke();

        // Location shown as a cloud sized by the 1-sigma range uncertainty: small + solid
        // when confident, large + diffuse when uncertain (fixed ink spread over more area).
        const st = statusRef.current;
        const sigPx = clamp((st.hasTagEst ? st.tagRangeSigma : 0.05) * scale, 5, 180);
        const coreA = clamp(1 - sigPx / 160, 0.22, 0.9);
        const grad = ctx.createRadialGradient(tsx, tsy, 0, tsx, tsy, sigPx);
        grad.addColorStop(0, `rgba(240,210,60,${coreA})`);
        grad.addColorStop(1, "rgba(240,210,60,0)");
        ctx.fillStyle = grad;
        ctx.beginPath(); ctx.arc(tsx, tsy, sigPx, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = "rgba(246,226,122,0.95)";   // crisp center dot marks the point estimate
        ctx.beginPath(); ctx.arc(tsx, tsy, 2.5, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = "#f6e27a"; ctx.font = "12px system-ui"; ctx.fillText("tag", tsx + sigPx + 6, tsy + 4);
      }

      drawCar(ctx, toS, car, scale, statusRef.current.steering);
      if (uwb) drawAnchor(ctx, toS, uwb);   // sensor sits on top of the chassis
      if (goal && goalFresh) drawGoal(ctx, toS, goal);   // committed goal on top, always visible

      if (!tagFresh) banner(ctx, w, "tag: no fix");
    };

    raf = requestAnimationFrame(render);
    return () => { cancelAnimationFrame(raf); canvas.removeEventListener("wheel", onWheel); };
  }, [treeRef, statusRef]);

  return <canvas ref={canvasRef} className="viewport" />;
}

// Faint 1 m grid across the visible region, brighter on the odom axes.
function drawGrid(ctx: CanvasRenderingContext2D, w: number, h: number, cx: number, cy: number, scale: number) {
  const halfW = w / 2 / scale, halfH = h / 2 / scale;
  const sx = (wx: number) => w / 2 + (wx - cx) * scale;
  const sy = (wy: number) => h / 2 - (wy - cy) * scale;
  ctx.lineWidth = 1;
  for (let gx = Math.floor(cx - halfW); gx <= Math.ceil(cx + halfW); gx++) {
    ctx.strokeStyle = gx === 0 ? "rgba(200,200,210,0.30)" : "rgba(120,130,140,0.13)";
    ctx.beginPath(); ctx.moveTo(sx(gx), 0); ctx.lineTo(sx(gx), h); ctx.stroke();
  }
  for (let gy = Math.floor(cy - halfH); gy <= Math.ceil(cy + halfH); gy++) {
    ctx.strokeStyle = gy === 0 ? "rgba(200,200,210,0.30)" : "rgba(120,130,140,0.13)";
    ctx.beginPath(); ctx.moveTo(0, sy(gy)); ctx.lineTo(w, sy(gy)); ctx.stroke();
  }
}

// Rounded-rectangle subpath (manual, so it doesn't depend on ctx.roundRect availability).
function roundRectPath(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number) {
  const rr = Math.max(0, Math.min(r, w / 2, h / 2));
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
  ctx.closePath();
}

// Stylized top-down chassis (outline + flat colors): four large tires with the front pair
// steering side-to-side, a chassis plate, a colored body shell, and a front bumper marking
// the nose (no arrow — front is the bumper/steering end).
function drawCar(ctx: CanvasRenderingContext2D, toS: ToScreen, car: Tf2D, scale: number, steering: number) {
  const [sx, sy] = toS(car.x, car.y);
  const L = CAR_L * scale, W = CAR_W * scale;
  const steerRad = clamp(steering, -1, 1) * (STEER_VIS_DEG * Math.PI / 180);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-car.yaw);                     // canvas y is down, negate odom yaw

  // Tires — large rounded rects at the corners; the front pair rotates with steering.
  const tL = L * 0.32, tW = W * 0.52;
  const wx = L * 0.36, wy = W / 2 + tW * 0.38;   // outboard of the chassis so they stick out
  const tire = (px: number, py: number, turn: number) => {
    ctx.save(); ctx.translate(px, py); ctx.rotate(-turn);
    ctx.fillStyle = "#17191d"; ctx.strokeStyle = "rgba(210,214,220,0.85)"; ctx.lineWidth = 1.4;
    roundRectPath(ctx, -tL / 2, -tW / 2, tL, tW, Math.min(tL, tW) * 0.3); ctx.fill(); ctx.stroke();
    ctx.restore();
  };
  tire(-wx, -wy, 0);                        // rear — fixed
  tire(-wx, wy, 0);
  tire(wx, -wy, steerRad);                  // front — steer side to side
  tire(wx, wy, steerRad);

  // Chassis plate.
  ctx.fillStyle = "#363c44"; ctx.strokeStyle = "rgba(220,224,230,0.55)"; ctx.lineWidth = 1.6;
  roundRectPath(ctx, -L / 2, -W / 2, L, W, Math.min(L, W) * 0.24); ctx.fill(); ctx.stroke();

  // Front bumper — marks the nose without an arrow.
  ctx.fillStyle = "#5b636d"; ctx.strokeStyle = "rgba(220,224,230,0.5)"; ctx.lineWidth = 1.2;
  roundRectPath(ctx, L * 0.34, -W * 0.34, L * 0.12, W * 0.68, 3); ctx.fill(); ctx.stroke();

  // Body shell (solid color), rear-biased so the front stays open.
  const bL = L * 0.6, bW = W * 0.76;
  ctx.fillStyle = "#d24b3e"; ctx.strokeStyle = "rgba(240,240,245,0.55)"; ctx.lineWidth = 2;
  roundRectPath(ctx, -L * 0.36, -bW / 2, bL, bW, Math.min(bL, bW) * 0.28); ctx.fill(); ctx.stroke();

  ctx.restore();
}

// UWB anchor drawn as a sensor rectangle at uwb_link, oriented with the pan servo; a short
// tick shows where the anchor is aimed.
function drawAnchor(ctx: CanvasRenderingContext2D, toS: ToScreen, uwb: Tf2D) {
  const [sx, sy] = toS(uwb.x, uwb.y);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-uwb.yaw);
  ctx.strokeStyle = "rgba(90,210,235,0.6)"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(11, 0); ctx.lineTo(34, 0); ctx.stroke();   // aim tick
  ctx.fillStyle = "#3ad0e6"; ctx.strokeStyle = "#0f6b78"; ctx.lineWidth = 2;
  roundRectPath(ctx, -11, -7, 22, 14, 2); ctx.fill(); ctx.stroke();      // sensor board
  ctx.restore();
  ctx.fillStyle = "#9fe6f2"; ctx.font = "11px system-ui"; ctx.fillText("uwb", sx + 13, sy - 11);
}

// Committed follow goal (nav_goal) as a green target: ring + crosshair + center dot.
// Fixed pixel size so it stays legible at any zoom.
function drawGoal(ctx: CanvasRenderingContext2D, toS: ToScreen, goal: Tf2D) {
  const [sx, sy] = toS(goal.x, goal.y);
  ctx.strokeStyle = "#46c76a"; ctx.fillStyle = "#46c76a"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(sx, sy, 11, 0, Math.PI * 2); ctx.stroke();          // ring
  ctx.beginPath();                                                             // crosshair
  ctx.moveTo(sx - 16, sy); ctx.lineTo(sx - 4, sy);
  ctx.moveTo(sx + 4, sy); ctx.lineTo(sx + 16, sy);
  ctx.moveTo(sx, sy - 16); ctx.lineTo(sx, sy - 4);
  ctx.moveTo(sx, sy + 4); ctx.lineTo(sx, sy + 16);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(sx, sy, 2.5, 0, Math.PI * 2); ctx.fill();           // center dot
  ctx.fillStyle = "#a7e7ba"; ctx.font = "11px system-ui"; ctx.fillText("goal", sx + 15, sy - 11);
}

// Small top-center status note.
function banner(ctx: CanvasRenderingContext2D, w: number, text: string) {
  ctx.fillStyle = "#e0b13c"; ctx.font = "13px system-ui"; ctx.textAlign = "center";
  ctx.fillText(text, w / 2, 20); ctx.textAlign = "left";
}
