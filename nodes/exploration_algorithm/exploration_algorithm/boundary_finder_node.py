import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from collections import deque
import numpy as np
import math, heapq, threading, time, sys

# Optional smoothing (graceful fallback if SciPy not available)
try:
    import scipy.interpolate as si
except Exception:
    si = None
    Node().get_logger().warn("[WARN] SciPy not available; BSpline smoothing will be disabled")

# Try to load params from YAML if present; else defaults
try:
    import yaml
    with open("src/autonomous_exploration/config/params.yaml", 'r') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)
except Exception:
    params = {}
    Node().get_logger().warn("[WARN] Failed to load params.yaml; using defaults")

# Motion/control params
LOOKAHEAD_DISTANCE = params.get("lookahead_distance", 0.24)
SPEED = params.get("speed", 0.18)
EXPANSION_SIZE = params.get("expansion_size", 6)    # obstacle inflation (cells)
TARGET_ERROR = params.get("target_error", 0.20)     # goal proximity (meters)
ROBOT_R = params.get("robot_r", 0.3)                # robot radius (meters)

# Planning params
MAX_LOOP_POINTS = params.get("max_loop_points", 120)  # subsample the loop to this many points (max)
FRONTIER_SUBSAMPLE = params.get("frontier_subsample", 2)  # take every Nth frontier cell before A*
REPLAN_EARLY_SEC = params.get("replan_early_sec", 0.2)

# ------------- Helper functions provided by user (do not modify semantics) -------------
def get_reachable_mask(grid: np.ndarray, position: tuple[int, int]) -> np.ndarray:
    h, w = grid.shape
    visited = np.zeros_like(grid, dtype=bool)
    q = deque([position])
    if not (0 <= position[0] < w and 0 <= position[1] < h):
        return visited
    visited[position[1], position[0]] = True

    while q:
        x, y = q.popleft()
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                if not visited[ny, nx] and grid[ny, nx] in [0, -1]:
                    visited[ny, nx] = True
                    q.append((nx, ny))

    return visited

def is_fully_enclosed(grid: np.ndarray, position: tuple[int, int]) -> bool:
    reachable = get_reachable_mask(grid, position)
    unknown_mask = (grid == -1)
    return not np.any(reachable & unknown_mask)
# --------------------------------------------------------------------------------------


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def astar(grid01, start, goal):
    """
    A* on binary grid (0 free, 1 blocked).
    Nodes are (row, col).
    """
    H, W = grid01.shape
    nbrs = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    closed = set()
    came = {}
    g = {start: 0.0}
    f = {start: heuristic(start, goal)}
    openh = []
    heapq.heappush(openh, (f[start], start))

    while openh:
        _, cur = heapq.heappop(openh)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path

        closed.add(cur)
        ci, cj = cur
        for di, dj in nbrs:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < H and 0 <= nj < W):
                continue
            if grid01[ni, nj] == 1:
                continue
            step_cost = math.hypot(di, dj)
            tentative_g = g[cur] + step_cost
            if (ni, nj) in closed and tentative_g >= g.get((ni, nj), float('inf')):
                continue
            if tentative_g < g.get((ni, nj), float('inf')) or (ni, nj) not in [x[1] for x in openh]:
                came[(ni, nj)] = cur
                g[(ni, nj)] = tentative_g
                f[(ni, nj)] = tentative_g + heuristic((ni, nj), goal)
                heapq.heappush(openh, (f[(ni, nj)], (ni, nj)))
    # closest reconstruction
    closest = None
    bestf = float('inf')
    for k, val in f.items():
        if val < bestf:
            bestf = val
            closest = k
    if closest is None:
        return None
    path = [closest]
    while closest in came:
        closest = came[closest]
        path.append(closest)
    path.reverse()
    return path


def bspline_planning(path, samples):
    if si is None:
        return path
    if path is None or len(path) < 3:
        return path
    try:
        arr = np.array(path, dtype=float)
        x = arr[:, 0]
        y = arr[:, 1]
        t = range(len(x))
        k = min(3, max(1, len(x) - 1))
        tx = si.splrep(t, x, k=k)
        ty = si.splrep(t, y, k=k)
        tt = np.linspace(0.0, len(x) - 1, samples)
        rx = si.splev(tt, tx)
        ry = si.splev(tt, ty)
        return [(rx[i], ry[i]) for i in range(len(rx))]
    except Exception:
        return path


def costmap_inflate(data, width, height, expansion_cells):
    """
    Inflate obstacles: keep -1 unknown, set 100 on neighbors within EXPANSION_SIZE.
    """
    grid = np.array(data, dtype=np.int16).reshape(height, width)
    occ_y, occ_x = np.where(grid == 100)
    if len(occ_y) > 0:
        for di in range(-expansion_cells, expansion_cells + 1):
            for dj in range(-expansion_cells, expansion_cells + 1):
                if di == 0 and dj == 0:
                    continue
                yi = np.clip(occ_y + di, 0, height - 1)
                xj = np.clip(occ_x + dj, 0, width - 1)
                grid[yi, xj] = 100
    return grid


def ensure_raw_semantics(inflated, raw):
    """
    Ensure grid semantics:
    - unknown from raw stays -1
    - free stays 0
    - everything else 100
    """
    g = inflated.copy()
    unk = (raw == -1)
    g[unk] = -1
    g[(g != -1) & (g != 0)] = 100
    return g


def reachable_free_mask(grid, start_rc):
    """
    BFS flood only over free=0 cells from start_rc (row, col) to restrict frontier to reachable area.
    """
    H, W = grid.shape
    mask = np.zeros((H, W), dtype=bool)
    si, sj = start_rc
    if not (0 <= si < H and 0 <= sj < W):
        return mask
    if grid[si, sj] != 0:
        return mask
    q = deque([(si, sj)])
    mask[si, sj] = True
    while q:
        i, j = q.popleft()
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = i + di, j + dj
            if 0 <= ni < H and 0 <= nj < W and not mask[ni, nj] and grid[ni, nj] == 0:
                mask[ni, nj] = True
                q.append((ni, nj))
    return mask


def compute_frontier_mask(grid, reachable_mask):
    """
    Frontier: reachable free cell (0) that has a 4-neighbor unknown (-1).
    """
    H, W = grid.shape
    fr = np.zeros((H, W), dtype=np.uint8)
    for i in range(H):
        for j in range(W):
            if grid[i, j] == 0 and reachable_mask[i, j]:
                if (i > 0 and grid[i-1, j] == -1) or \
                   (i < H-1 and grid[i+1, j] == -1) or \
                   (j > 0 and grid[i, j-1] == -1) or \
                   (j < W-1 and grid[i, j+1] == -1):
                    fr[i, j] = 1
    return fr


def nearest_frontier(frontier_mask, start_rc):
    ys, xs = np.where(frontier_mask > 0)
    if len(ys) == 0:
        return None
    si, sj = start_rc
    d2 = (ys - si) * (ys - si) + (xs - sj) * (xs - sj)
    k = int(np.argmin(d2))
    return (int(ys[k]), int(xs[k]))


def moore_trace_ccw(frontier_mask, grid):
    """
    Trace a CCW loop along frontier cells using a left-hand (unknown-on-left) rule.
    Returns an ordered list of (row, col) forming a loop; may be open if break occurs.
    """
    H, W = frontier_mask.shape
    points = list(zip(*np.where(frontier_mask > 0)))
    if not points:
        return []

    # Start at the frontier point with minimal (row, col) for determinism
    start = min(points)
    # If we have a better start (nearest to an unknown), pick it
    def has_unknown_left(i, j, d):
        # left direction relative to heading d
        dirs = [(0,1),(1,0),(0,-1),(-1,0)]  # E,S,W,N
        li, lj = dirs[(d - 1) % 4]
        ui, uj = i + li, j + lj
        return 0 <= ui < H and 0 <= uj < W and grid[ui, uj] == -1

    # Choose initial heading so that unknown is on the left
    # Try all four; pick the one that has unknown on left and a frontier ahead
    dirs = [(0,1),(1,0),(0,-1),(-1,0)]  # E,S,W,N
    d = 0
    chosen = False
    for cand in range(4):
        if has_unknown_left(start[0], start[1], cand):
            fi, fj = start[0] + dirs[cand][0], start[1] + dirs[cand][1]
            if 0 <= fi < H and 0 <= fj < W and frontier_mask[fi, fj] > 0:
                d = cand
                chosen = True
                break
    if not chosen:
        # fallback: pick any direction that keeps unknown on left
        for cand in range(4):
            if has_unknown_left(start[0], start[1], cand):
                d = cand
                chosen = True
                break

    path = [start]
    cur = start
    max_steps = max(2000, len(points) * 8)
    visited_times = {start: 1}

    for _ in range(max_steps):
        # left, forward, right, back preference to keep unknown on left (CCW)
        for turn in [-1, 0, +1, +2]:
            nd = (d + turn) % 4
            ni, nj = cur[0] + dirs[nd][0], cur[1] + dirs[nd][1]
            if 0 <= ni < H and 0 <= nj < W and frontier_mask[ni, nj] > 0:
                # Ensure unknown remains on left where possible
                if has_unknown_left(ni, nj, nd) or turn != -1:
                    cur = (ni, nj)
                    d = nd
                    path.append(cur)
                    visited_times[cur] = visited_times.get(cur, 0) + 1
                    break
        else:
            # No neighbor frontier found
            break

        # Closed loop detection: back at start and progressed enough
        if cur == start and len(path) > 10:
            break
        # Avoid infinite cycling on tiny loops
        if visited_times.get(cur, 0) > 4:
            break

    # Deduplicate consecutive duplicates
    out = []
    for p in path:
        if not out or out[-1] != p:
            out.append(p)
    return out


def order_ccw_by_angle(points):
    """
    Order a set of (row,col) points counter-clockwise by angle around their centroid.
    """
    if not points:
        return []
    ci = sum(p[0] for p in points) / len(points)
    cj = sum(p[1] for p in points) / len(points)
    # Image rows increase downward; flip row to y = -row for proper CCW in world
    ordered = sorted(points, key=lambda p: math.atan2(-(p[0]-ci), (p[1]-cj)))
    return ordered


def ensure_ccw_world(path_grid, res, ox, oy):
    """
    Ensure the closed path has CCW orientation in world coordinates (x right, y up).
    Reverse if clockwise.
    """
    if not path_grid or len(path_grid) < 3:
        return path_grid
    pts = [(c*res + ox, r*res + oy) for (r, c) in path_grid]
    # Signed area (shoelace): >0 => CCW
    area = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i+1) % len(pts)]
        area += (x1 * y2 - x2 * y1)
    if area < 0:
        return list(reversed(path_grid))
    return path_grid


def grid_to_world(path_grid, res, ox, oy):
    return [(c * res + ox, r * res + oy) for (r, c) in path_grid]


def path_length(path_world):
    if not path_world or len(path_world) < 2:
        return 0.0
    dist = 0.0
    for i in range(1, len(path_world)):
        x0, y0 = path_world[i-1]
        x1, y1 = path_world[i]
        dist += math.hypot(x1 - x0, y1 - y0)
    return dist


def pure_pursuit(x, y, yaw, path, idx):
    """
    Basic pure pursuit to follow the world-frame path.
    """
    if not path:
        return 0.0, 0.0, idx
    v = SPEED
    target = None
    for i in range(idx, len(path)):
        px, py = path[i]
        d = math.hypot(px - x, py - y)
        if d > LOOKAHEAD_DISTANCE:
            target = (px, py)
            idx = i
            break
    if target is None:
        target = path[-1]
        idx = len(path) - 1
    tx, ty = target
    th = math.atan2(ty - y, tx - x)
    err = th - yaw
    # Normalize
    while err > math.pi:
        err -= 2.0 * math.pi
    while err < -math.pi:
        err += 2.0 * math.pi
    # Slow for sharp turns
    if abs(err) > math.pi/6:
        v = 0.0
        err = math.copysign(math.pi/4, err)
    return v, err, idx


def stitch_with_astar(binary_grid, seq_cells):
    """
    Connect a sequence of grid cells (row,col) with A* segments.
    """
    if not seq_cells:
        return None
    out = []
    for k in range(len(seq_cells)-1):
        a = seq_cells[k]
        b = seq_cells[k+1]
        seg = astar(binary_grid, a, b)
        if seg is None or len(seg) == 0:
            continue
        if not out:
            out.extend(seg)
        else:
            if out[-1] == seg[0]:
                out.extend(seg[1:])
            else:
                out.extend(seg)
    return out if out else None


def build_boundary_path_ccw(occ_grid_msg, start_rc):
    """
    Build a CCW wall-hugging path along the outer frontier near the robot.
    Returns:
      -1 if fully enclosed (stop condition)
       list of world (x,y) waypoints otherwise (may be None if not ready)
    """
    W = occ_grid_msg.info.width
    H = occ_grid_msg.info.height
    res = occ_grid_msg.info.resolution
    ox = occ_grid_msg.info.origin.position.x
    oy = occ_grid_msg.info.origin.position.y

    raw = np.array(occ_grid_msg.data, dtype=np.int16).reshape(H, W)

    # Stop if boundary is fully enclosed per provided function
    # Note: function expects (x, y) -> (col, row)
    if is_fully_enclosed(raw, (start_rc[1], start_rc[0])):
        return -1

    # Inflate obstacles by ROBOT_R in meters, converted to cells
    res = occ_grid_msg.info.resolution
    robot_radius_cells = max(1, int(math.ceil(ROBOT_R / res)))
    inflated = costmap_inflate(occ_grid_msg.data, W, H, max(EXPANSION_SIZE, robot_radius_cells))
    grid = ensure_raw_semantics(inflated, raw)  # -1 unknown, 0 free, 100 occ

    si, sj = start_rc
    if not (0 <= si < H and 0 <= sj < W) or grid[si, sj] != 0:
        # Snap to nearest free within a small radius
        found = None
        for rad in range(1, 8):
            for di in range(-rad, rad+1):
                for dj in range(-rad, rad+1):
                    ni, nj = si + di, sj + dj
                    if 0 <= ni < H and 0 <= nj < W and grid[ni, nj] == 0:
                        found = (ni, nj)
                        break
                if found:
                    break
            if found:
                break
        if not found:
            return None
        start_rc = found

    reachable = reachable_free_mask(grid, start_rc)
    frontier = compute_frontier_mask(grid, reachable)
    if frontier.sum() == 0:
        return None

    # Trace a CCW loop; fallback to angle-ordering if tracing fails
    loop_cells = moore_trace_ccw(frontier, grid)
    if not loop_cells or len(loop_cells) < 5:
        pts = list(zip(*np.where(frontier > 0)))
        if not pts:
            return None
        loop_cells = order_ccw_by_angle(pts)

    # Make sure CCW in world frame
    loop_cells = ensure_ccw_world(loop_cells, res, ox, oy)

    # Subsample to reduce density for stitching and tracking
    if FRONTIER_SUBSAMPLE > 1:
        loop_cells = loop_cells[::FRONTIER_SUBSAMPLE]
    if len(loop_cells) > MAX_LOOP_POINTS:
        step = max(1, len(loop_cells) // MAX_LOOP_POINTS)
        loop_cells = loop_cells[::step]

    # Build binary grid for A* (unknown and occ blocked)
    bin_grid = np.zeros_like(grid, dtype=np.uint8)
    bin_grid[grid != 0] = 1  # block unknown and occ

    # Connect the loop with A*
    stitched = stitch_with_astar(bin_grid, loop_cells)
    if stitched is None or len(stitched) < 2:
        # Fallback: go to nearest frontier cell only
        nf = nearest_frontier(frontier, start_rc)
        if nf is None:
            return None
        stitched = astar(bin_grid, start_rc, nf)
        if stitched is None:
            return None

    # Convert to world and smooth
    world_path = grid_to_world(stitched, res, ox, oy)
    world_path = bspline_planning(world_path, max(10, len(world_path)*3))
    return world_path


class BoundaryExplorer(Node):
    def __init__(self):
        super().__init__("boundary_explorer_ccw")
        self.sub_map = self.create_subscription(OccupancyGrid, "map", self.on_map, 10)
        self.sub_odom = self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)

        self.map_msg = None
        self.odom_msg = None
        self.path = None
        self.idx = 0
        self.following = False
        self.replan_timer = None

        self.get_logger().info("[INFO] Boundary-exploration (CCW wall-hugging) active")
        threading.Thread(target=self.spin_loop, daemon=True).start()

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height

    def on_odom(self, msg: Odometry):
        self.odom_msg = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w)

    def spin_loop(self):
        tw = Twist()
        while rclpy.ok():
            if self.map_msg is None or self.odom_msg is None:
                time.sleep(0.05)
                continue

            # Convert robot world pose to grid indices (row, col)
            row = int((self.y - self.originY) / self.resolution)
            col = int((self.x - self.originX) / self.resolution)
            start_rc = (row, col)

            # Planning phase
            if not self.following:
                try:
                    plan = build_boundary_path_ccw(self.map_msg, start_rc)
                except Exception as e:
                    self.get_logger().warn(f"Planning exception: {e}")
                    plan = None

                if isinstance(plan, int) and plan == -1:
                    self.get_logger().info("[INFO] Exploration finished: boundary fully enclosed")
                    tw.linear.x = 0.0
                    tw.angular.z = 0.0
                    self.pub_cmd.publish(tw)
                    time.sleep(0.2)
                    sys.exit(0)

                if plan is None or len(plan) < 2:
                    # No plan yet; hold
                    tw.linear.x = 0.0
                    tw.angular.z = 0.0
                    self.pub_cmd.publish(tw)
                    time.sleep(0.1)
                    continue

                self.path = plan
                self.idx = 0
                self.following = True

                # Estimate traversal time and schedule early replan
                T = max(0.5, path_length(self.path) / max(1e-3, SPEED) - REPLAN_EARLY_SEC)
                if self.replan_timer is not None:
                    try:
                        self.replan_timer.cancel()
                    except Exception:
                        pass
                self.replan_timer = threading.Timer(T, self.request_replan)
                self.replan_timer.daemon = True
                self.replan_timer.start()
                self.get_logger().info("[INFO] New CCW boundary segment set")

            # Control phase: follow the path
            v, w, self.idx = pure_pursuit(self.x, self.y, self.yaw, self.path, self.idx)

            # End segment if we reach the end of current path
            if self.path and abs(self.x - self.path[-1][0]) < TARGET_ERROR and abs(self.y - self.path[-1][1]) < TARGET_ERROR:
                v, w = 0.0, 0.0
                self.following = False
                if self.replan_timer:
                    self.replan_timer.join(timeout=0.1)
                self.get_logger().info("[INFO] Reached segment end; replanning")

            tw.linear.x = v
            tw.angular.z = w
            self.pub_cmd.publish(tw)
            time.sleep(0.1)

    def request_replan(self):
        self.following = False


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()