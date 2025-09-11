import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import heapq
import json
import logging
import math
import numpy as np
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
import scipy.interpolate as si


# Constants for occupancy values
VAL_UNKNOWN = -1
VAL_FREE = 0
VAL_OCCUPIED = 100

# Dumping/overlay values
# Next exploration target cell will be 20..29 (cycling)
# Visible cells from that next target will be 30..39 (cycling)
DUMP_TARGET_BASE = 20
DUMP_VIEW_BASE = 30


class AreaExploration(Node):
    """
    ROS2 node that plans exploration paths in unknown environments using A* to
    a sequence of discrete exploration points. Each next point is chosen to
    maximize newly-visible unknown cells given a 180-degree FOV and
    lookahead_distance range constraint. The path is re-planned on each /map update.
    """
    def __init__(self):
        super().__init__('boundary_finder_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lookahead_distance', 0.5),   # meters
                ('speed', 0.15),               # m/s
                ('expansion_size', 3),         # obstacle inflation (in cells)
                ('target_error', 0.1),         # goal tolerance (meters)
                ('robot_r', 0.18),             # collision radius (meters)
                ('resolution', 0.05),          # fallback, will be overwritten by map
                ('map_dump', True),
                ('output_directory', 'messages'),
                ('log_level', 'INFO'),
                ('laser_max_range', 1.0)
            ]
        )

        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.expansion_size = self.get_parameter('expansion_size').get_parameter_value().integer_value
        self.target_error = self.get_parameter('target_error').get_parameter_value().double_value
        self.robot_r = self.get_parameter('robot_r').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.map_dump = self.get_parameter('map_dump').get_parameter_value().bool_value
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        self.laser_max_range = self.get_parameter('laser_max_range').get_parameter_value().double_value

        # Logging level
        level = self.get_parameter('log_level').get_parameter_value().string_value.upper()
        self.get_logger().set_level(getattr(logging, level, logging.INFO))

        # Create output dir
        if self.map_dump and not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory, exist_ok=True)

        # ROS interfaces
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # State
        self.map_msg: OccupancyGrid | None = None
        self.odom_msg: Odometry | None = None
        self.scan_msg: LaserScan | None = None

        self.grid = None          # planning grid: 0 free, 1 blocked (inflated)
        self.raw_grid = None      # raw occupancy values from map (integers)
        self.width = None
        self.height = None
        self.originX = None
        self.originY = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_world: list[tuple[float, float]] = []  # current planned path (world coords)
        self.path_idx = 0

        # Dump cycle index [0..9] for 20..29 and 30..39
        self.dump_cycle = 0

        # For dumping
        self.next_target_rc = None
        self.view_cells_for_dump = set()

        # Multi-goal overlays (list of (rc, visibility_set))
        self.future_targets = []

        # Control loop
        threading.Thread(target=self.control_loop, daemon=True).start()
        self.get_logger().info("Area exploration node initialized.")

    # ==================== ROS Callbacks ====================

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y

        # Build raw and planning grids
        raw = np.array(msg.data, dtype=np.int16).reshape(self.height, self.width)
        self.raw_grid = raw
        self.grid = self.build_planning_grid(raw)

        # Re-plan path on each map update if we have odom
        if self.odom_msg is not None:
            self.plan_next_path()

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    # ==================== Planning ====================

    def build_planning_grid(self, raw_grid: np.ndarray) -> np.ndarray:
        """
        Returns a binary grid used for planning:
        - 1 for blocked (inflated obstacles)
        - 0 for traversable (free or unknown)
        """
        h, w = raw_grid.shape
        blocked = (raw_grid >= 50)  # occupied thresholds
        # obstacle inflation by square neighborhood expansion_size
        if self.expansion_size > 0:
            inflated = blocked.copy()
            # fast binary dilation without external deps: sliding window
            # pad
            pad = self.expansion_size
            padded = np.pad(blocked, pad_width=pad, mode='edge')
            # accumulate max over neighborhood
            for di in range(-pad, pad + 1):
                for dj in range(-pad, pad + 1):
                    inflated |= padded[pad + di: pad + di + h, pad + dj: pad + dj + w]
            blocked = inflated
        grid = np.zeros_like(raw_grid, dtype=np.uint8)
        grid[blocked] = 1
        # free and unknown remain 0
        return grid

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        c = int((x - self.originX) / self.resolution)
        r = int((y - self.originY) / self.resolution)
        # clamp to bounds
        r = max(0, min(self.height - 1, r))
        c = max(0, min(self.width - 1, c))
        return r, c

    def grid_to_world(self, r: int, c: int) -> tuple[float, float]:
        x = c * self.resolution + self.originX
        y = r * self.resolution + self.originY
        return x, y

    def heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def astar(self, grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        """
        A* on 8-connected grid. grid: 0 free, 1 blocked
        Returns list of (r, c) including start..goal, or None.
        Allows traversal through unknown (-1) as they are 0 in grid.
        """
        if grid[goal] == 1:
            return None
        h, w = grid.shape
        neighbors = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        open_heap = []
        g = {start: 0.0}
        f = {start: self.heuristic(start, goal)}
        came = {}
        heapq.heappush(open_heap, (f[start], start))
        closed = set()
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur == goal:
                # reconstruct
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return list(reversed(path))
            if cur in closed:
                continue
            closed.add(cur)
            cr, cc = cur
            for dr, dc in neighbors:
                nr, nc = cr + dr, cc + dc
                if nr < 0 or nr >= h or nc < 0 or nc >= w:
                    continue
                if grid[nr, nc] == 1:
                    continue
                step = math.hypot(dr, dc)
                tentative = g[cur] + step
                if tentative < g.get((nr, nc), float('inf')):
                    came[(nr, nc)] = cur
                    g[(nr, nc)] = tentative
                    f[(nr, nc)] = tentative + self.heuristic((nr, nc), goal)
                    heapq.heappush(open_heap, (f[(nr, nc)], (nr, nc)))
        return None

    def plan_next_path(self):
        """
        Plan a multi-goal exploration path up to 10 future goals.
        For each step, choose the goal that maximizes simulated new coverage (unknown cells seen)
        within a 180-degree FOV and within a lookahead radius. For steps > 1, the coverage
        is evaluated on a simulated map where cells revealed by prior selected goals are treated
        as known (no longer unknown). The final path concatenates A* paths between successive goals.
        Also prepares overlays for dumping: the i-th future goal is marked with 20+i and its
        simulated coverage with 30+i.
        """
        if self.grid is None or self.raw_grid is None:
            return

        h, w = self.grid.shape
        cur_rc = self.world_to_grid(self.x, self.y)

        # If robot already close to previous terminal goal, clear path for fresh planning
        if self.path_world:
            goal_x, goal_y = self.path_world[-1]
            if abs(self.x - goal_x) < self.target_error and abs(self.y - goal_y) < self.target_error:
                self.path_world = []
                self.path_idx = 0

        lhd_cells = max(1, int(round(self.laser_max_range / self.resolution)))

        # The simulated set of unknown cells that remain unexplored
        unknown_set = {(r, c) for r in range(h) for c in range(w) if self.raw_grid[r, c] == VAL_UNKNOWN and self.grid[r, c] == 0}

        # Helper to generate candidate unknown cells around a reference cell
        def generate_candidates(ref_rc: tuple[int, int]) -> set[tuple[int, int]]:
            candidates = set()
            radii = [lhd_cells, int(1.5 * lhd_cells), 2 * lhd_cells, int(2.5 * lhd_cells), 3 * lhd_cells]
            rr0, cc0 = ref_rc
            for r_cells in radii:
                for deg in range(0, 360, 10):
                    ang = math.radians(deg)
                    rr = int(round(rr0 + r_cells * math.sin(ang)))
                    cc = int(round(cc0 + r_cells * math.cos(ang)))
                    if 0 <= rr < h and 0 <= cc < w:
                        if (rr, cc) not in unknown_set:
                            found = None
                            for dr in range(-2, 3):
                                for dc in range(-2, 3):
                                    nr, nc = rr + dr, cc + dc
                                    if 0 <= nr < h and 0 <= nc < w and (nr, nc) in unknown_set:
                                        found = (nr, nc); break
                                if found: break
                            if found:
                                candidates.add(found)
                        else:
                            candidates.add((rr, cc))
            # Fallback coarse scan if still empty
            if not candidates:
                stride = max(1, lhd_cells // 2)
                for rr in range(0, h, stride):
                    for cc in range(0, w, stride):
                        if (rr, cc) in unknown_set:
                            candidates.add((rr, cc))
                            if len(candidates) > 200:
                                break
                    if len(candidates) > 200:
                        break
            return candidates

        # Multi-goal planning loop
        max_goals = 10
        future_targets: list[tuple[tuple[int, int], set[tuple[int, int]]]] = []
        full_path_rc: list[tuple[int, int]] = []
        ref_rc = cur_rc

        for step_idx in range(max_goals):
            candidates = generate_candidates(ref_rc)
            if not candidates:
                # No more unknowns
                break

            # Score candidates by simulated new coverage on current unknown_set
            scored = []
            for cand in candidates:
                if self.grid_distance(ref_rc, cand) < lhd_cells:
                    continue
                heading = math.atan2(cand[0] - ref_rc[0], cand[1] - ref_rc[1])
                vis = self.simulate_visibility(cand, heading, fov_rad=math.pi, radius_cells=lhd_cells)
                cov = sum(1 for rc in vis if rc in unknown_set)
                if cov <= 0:
                    continue
                scored.append((cov, cand, vis))

            path_segment = None
            chosen = None

            if not scored:
                # If no coverage gain, still push towards nearest unknown that is reachable
                fallback = sorted(list(candidates), key=lambda rc: self.grid_distance(ref_rc, rc))
                for cand in fallback[:50]:
                    seg = self.astar(self.grid, ref_rc, cand)
                    if seg is not None:
                        chosen = (0, cand, set())  # zero additional coverage, empty vis set
                        path_segment = seg
                        break
                if chosen is None:
                    # Can't reach anything from here
                    break
            else:
                scored.sort(key=lambda t: t[0], reverse=True)
                topK = scored[:20]
                best = None
                for cov, cand, vis in topK:
                    seg = self.astar(self.grid, ref_rc, cand)
                    if seg is None:
                        continue
                    plen = len(seg)
                    if best is None or (cov, -plen) > (best[0], -best[3]):
                        best = (cov, cand, vis, plen, seg)
                if best is None:
                    # Try broader set
                    for cov, cand, vis in scored:
                        seg = self.astar(self.grid, ref_rc, cand)
                        if seg is not None:
                            best = (cov, cand, vis, len(seg), seg)
                            break
                if best is None:
                    # No reachable candidate
                    break
                chosen = (best[0], best[1], set(best[2]))
                path_segment = best[4]

            # Append path segment (avoid duplicating the starting ref cell if already present)
            if not full_path_rc:
                full_path_rc.extend(path_segment)
            else:
                full_path_rc.extend(path_segment[1:])

            # Record chosen target and its visibility (for overlays)
            _, cand_rc, vis_set = chosen
            future_targets.append((cand_rc, vis_set))

            # Update simulation state for next step
            for rc in vis_set:
                if rc in unknown_set:
                    unknown_set.remove(rc)
            ref_rc = cand_rc

        # If we planned at least one step, convert to world and smooth
        if full_path_rc:
            path_world = [self.grid_to_world(r, c) for (r, c) in full_path_rc]
            self.path_world = self.bspline_planning(path_world, max(2, len(path_world) * 5))
            self.path_idx = 0
        else:
            # No path; clear
            self.path_world = []
            self.path_idx = 0

        # Save overlays for dumping
        print(f"Planned {len(future_targets)} exploration goals, path length {len(self.path_world)}")
        self.future_targets = future_targets
        if future_targets:
            self.next_target_rc = future_targets[0][0]
            self.view_cells_for_dump = set(future_targets[0][1])
        else:
            self.next_target_rc = None
            self.view_cells_for_dump = set()

        # Dump map with overlays
        self.dump_map_visualization()

    def simulate_visibility(self, center_rc: tuple[int, int], heading: float, fov_rad: float, radius_cells: int) -> set[tuple[int, int]]:
        """
        Simulate 2D visibility from center_rc within a sector defined by heading and fov_rad,
        limited to radius_cells. LOS blocked by occupied cells (grid==1).
        Returns a set of visible (r, c) cells.
        """
        visible = set()
        h, w = self.grid.shape
        # Cast rays every 2 degrees for speed
        step_deg = 2
        half = fov_rad / 2.0
        start = heading - half
        end = heading + half
        # Normalize angles
        def norm(a):
            while a <= -math.pi: a += 2 * math.pi
            while a > math.pi: a -= 2 * math.pi
            return a
        start = norm(start); end = norm(end)
        # Build list of sample angles covering the arc
        angles = []
        if start <= end:
            a = start
            while a <= end + 1e-6:
                angles.append(a)
                a += math.radians(step_deg)
        else:
            # wrapped
            a = start
            while a <= math.pi + 1e-6:
                angles.append(a)
                a += math.radians(step_deg)
            a = -math.pi
            while a <= end + 1e-6:
                angles.append(a)
                a += math.radians(step_deg)

        for ang in angles:
            # DDA ray march
            dr = math.sin(ang)
            dc = math.cos(ang)
            r, c = center_rc
            rr = r + 0.5  # center of cell
            cc = c + 0.5
            for step in range(radius_cells):
                rr += dr
                cc += dc
                ir = int(rr)
                ic = int(cc)
                if ir < 0 or ir >= h or ic < 0 or ic >= w:
                    break
                visible.add((ir, ic))
                # stop if blocked
                if self.grid[ir, ic] == 1:
                    break
        return visible

    def grid_distance(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    # ==================== Control ====================

    def control_loop(self):
        twist = Twist()
        rate = 0.05  # 20Hz
        while rclpy.ok():
            if self.path_world and self.path_idx < len(self.path_world):
                # Local obstacle check from laser; if close obstacle, turn away
                v_cmd, w_cmd = self.local_control(self.scan_msg.ranges if self.scan_msg else None)
                # v_cmd, w_cmd = self.local_control(self.scan_msg)
                
                if v_cmd is None:
                    v_cmd, w_cmd, self.path_idx = self.pure_pursuit(self.x, self.y, self.yaw, self.path_world, self.path_idx)
                twist.linear.x = v_cmd
                twist.angular.z = w_cmd
                # Check goal tolerance
                gx, gy = self.path_world[-1]
                if abs(self.x - gx) < self.target_error and abs(self.y - gy) < self.target_error:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    # Advance dump cycle on reaching a goal so next dump uses next code
                    self.dump_cycle = (self.dump_cycle + 1) % 10
                    # Clear to allow fresh planning on next map update
                    self.path_world = []
                    self.path_idx = 0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(rate)

    def pure_pursuit(self, current_x: float, current_y: float, current_heading: float,
                     path: list[tuple[float, float]], index: int) -> tuple[float, float, int]:
        v = self.speed
        lookahead = self.lookahead_distance
        closest = None
        for i in range(index, len(path)):
            px, py = path[i]
            if math.hypot(px - current_x, py - current_y) > lookahead:
                closest = (px, py); index = i; break
        if closest is None:
            px, py = path[-1]
            index = len(path) - 1
        else:
            px, py = closest
        target_heading = math.atan2(py - current_y, px - current_x)
        steer = target_heading - current_heading
        # wrap
        if steer > math.pi: steer -= 2 * math.pi
        if steer < -math.pi: steer += 2 * math.pi
        # limit sharp turns
        if abs(steer) > math.pi / 6:
            steer = math.copysign(math.pi / 4, steer)
            v = 0.0
        return v, steer, index

    # def local_control(self, scan_ranges) -> tuple[float | None, float | None]:
    #     if scan_ranges is None:
    #         return None, None
    #     # Simple reactive: if obstacle in ±30° very close, turn away
    #     n = len(scan_ranges)
    #     if n == 0:
    #         return None, None
    #     # 360 samples assumed; guard if different
    #     left_indices = range((n*5)//12, (n*7)//12)  # approx +75..+105 deg
    #     right_indices = range((n*5)//12 - (n//6), (n*7)//12 - (n//6))  # approx -105..-75 deg
    #     # center ±30 deg
    #     center_lo = (n*5)//12 - (n//12)
    #     center_hi = (n*7)//12 + (n//12)
    #     min_front = min([scan_ranges[i] for i in range(center_lo, min(center_hi, n)) if not math.isinf(scan_ranges[i])], default=float('inf'))
    #     if min_front < self.robot_r:
    #         # turn away
    #         return 0.2, -math.pi/4
    #     return None, None
    
    def local_control(self, scan) -> tuple[float | None, float | None]:
        v = None
        w = None
        for i in range(60):
            if scan[i] < self.robot_r:
                v = 0.2
                w = -math.pi/4 
                break
        if v == None:
            for i in range(300,360):
                if scan[i] < self.robot_r:
                    v = 0.2
                    w = math.pi/4
                    break
        return v,w

    # ==================== Utilities ====================

    def bspline_planning(self, array: list[tuple[float, float]], sn: int) -> list[tuple[float, float]]:
        try:
            array = np.array(array)
            x = array[:, 0]
            y = array[:, 1]
            N = 2
            t = range(len(x))
            x_tup = si.splrep(t, x, k=N)
            y_tup = si.splrep(t, y, k=N)
            x_list = list(x_tup); y_list = list(y_tup)
            x_list[1] = x.tolist() + [0.0, 0.0, 0.0, 0.0]
            y_list[1] = y.tolist() + [0.0, 0.0, 0.0, 0.0]
            ipl_t = np.linspace(0.0, len(x) - 1, sn)
            rx = si.splev(ipl_t, x_list)
            ry = si.splev(ipl_t, y_list)
            return [(rx[i], ry[i]) for i in range(len(rx))]
        except Exception:
            return array

    def euler_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
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

    # ==================== Dumping ====================

    def dump_map_visualization(self):
        if not self.map_dump or self.map_msg is None:
            return
        dumped = np.array(self.map_msg.data, dtype=np.int16).reshape(self.height, self.width)

        # If we have a sequence of future targets, overlay them with 20..29 for targets and 30..39 for coverage
        if hasattr(self, 'future_targets') and self.future_targets:
            for i, (trc, vis_set) in enumerate(self.future_targets[:10]):
                code_target = DUMP_TARGET_BASE + i  # 20..29
                code_view = DUMP_VIEW_BASE + i      # 30..39
                tr, tc = trc
                if 0 <= tr < self.height and 0 <= tc < self.width and dumped[vr, vc] == VAL_UNKNOWN:
                    dumped[tr, tc] = code_target
                for (vr, vc) in vis_set:
                    if 0 <= vr < self.height and 0 <= vc < self.width and dumped[vr, vc] == VAL_UNKNOWN:
                        dumped[vr, vc] = code_view
        else:
            # Legacy single-target overlay using dump_cycle
            if self.next_target_rc is not None:
                code_target = DUMP_TARGET_BASE + self.dump_cycle  # 20..29
                code_view = DUMP_VIEW_BASE + self.dump_cycle      # 30..39

                tr, tc = self.next_target_rc
                if 0 <= tr < self.height and 0 <= tc < self.width and dumped[vr, vc] == VAL_UNKNOWN:
                    dumped[tr, tc] = code_target

                for (vr, vc) in self.view_cells_for_dump:
                    if 0 <= vr < self.height and 0 <= vc < self.width and dumped[vr, vc] == VAL_UNKNOWN:
                        dumped[vr, vc] = code_view

        # Save JSON with overlays
        try:
            map_data = {
                'header': {
                    'stamp': {
                        'sec': self.map_msg.header.stamp.sec,
                        'nanosec': self.map_msg.header.stamp.nanosec
                    },
                    'frame_id': self.map_msg.header.frame_id
                },
                'info': {
                    'map_load_time': {
                        'sec': self.map_msg.info.map_load_time.sec,
                        'nanosec': self.map_msg.info.map_load_time.nanosec
                    },
                    'resolution': self.map_msg.info.resolution,
                    'width': self.map_msg.info.width,
                    'height': self.map_msg.info.height,
                    'origin': {
                        'position': {
                            'x': self.map_msg.info.origin.position.x,
                            'y': self.map_msg.info.origin.position.y,
                            'z': self.map_msg.info.origin.position.z
                        },
                        'orientation': {
                            'x': self.map_msg.info.origin.orientation.x,
                            'y': self.map_msg.info.origin.orientation.y,
                            'z': self.map_msg.info.origin.orientation.z,
                            'w': self.map_msg.info.origin.orientation.w
                        }
                    }
                },
                'data': dumped.flatten().tolist()
            }

            odom = self.odom_msg
            if odom is not None:
                odom_data = {
                    'header': {
                        'stamp': {
                            'sec': odom.header.stamp.sec,
                            'nanosec': odom.header.stamp.nanosec
                        },
                        'frame_id': odom.header.frame_id
                    },
                    'pose': {
                        'pose': {
                            'position': {
                                'x': odom.pose.pose.position.x,
                                'y': odom.pose.pose.position.y,
                                'z': odom.pose.pose.position.z
                            },
                            'orientation': {
                                'x': odom.pose.pose.orientation.x,
                                'y': odom.pose.pose.orientation.y,
                                'z': odom.pose.pose.orientation.z,
                                'w': odom.pose.pose.orientation.w
                            }
                        }
                    },
                }
            else:
                odom_data = None

            res_data = {'map': map_data}
            if odom_data is not None:
                res_data['odom'] = odom_data

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_directory, f'map_odom_{timestamp}.json')
            with open(filename, 'w') as f:
                json.dump(res_data, f, indent=2)
            self.get_logger().info(f"Saved map dump with overlays to {filename}")
        except Exception as e:
            self.get_logger().warning(f"Failed to dump map visualization: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AreaExploration()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()