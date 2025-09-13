from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from time import sleep

import math
import heapq
import json
import argparse
from typing import List, Tuple, Dict, Set, Optional
import numpy as np

# ---- occupancy conventions (ROS-like dumps) ----------------------------------
VAL_UNKNOWN = -1
VAL_FREE = 0
VAL_OCCUPIED = 100  # cells with 100 are obstacles (inflated)

DUMP_TARGET_BASE = 20  # 20..29
DUMP_VIEW_BASE   = 30  # 30..39

STATE: Dict[str, object] = {
    "grid": None,               # np.ndarray (H, W), int
    "height": None,             # int
    "width": None,              # int
    "resolution": None,         # float [m/cell]
    "origin_xy": None,          # (x0, y0) world coords of cell (0,0)
    "robot_xy": None,           # (x, y) world coords
    "robot_rc": None,           # (r, c) grid coords
    "robot_fov_deg": 270.0,     # default
    "laser_max_range": 0.5,     # meters
    "robot_max_speed": 0.18,     # m/s
}

def init_from_dump(
        json_path: str,
        robot_fov_deg: float = 120.0,
        laser_max_range: float = 5.0,
        robot_max_speed: float = 0.4
    ) -> None:
    """Load a dumped map/odom JSON (same structure as provided) and populate STATE."""
    with open(json_path, "r") as f:
        data = json.load(f)

    res = float(data["map"]["info"]["resolution"])
    width = int(data["map"]["info"]["width"])
    height = int(data["map"]["info"]["height"])
    x0 = float(data["map"]["info"]["origin"]["position"]["x"])
    y0 = float(data["map"]["info"]["origin"]["position"]["y"])

    grid = np.array(data["map"]["data"], dtype=int).reshape(height, width)

    rx = float(data["odom"]["pose"]["pose"]["position"]["x"])
    ry = float(data["odom"]["pose"]["pose"]["position"]["y"])

    rr = int((ry - y0) / res)
    rc = int((rx - x0) / res)

    STATE.update({
        "grid": grid,
        "height": height,
        "width": width,
        "resolution": res,
        "origin_xy": (x0, y0),
        "robot_xy": (rx, ry),
        "robot_rc": (rr, rc),
        "robot_fov_deg": float(robot_fov_deg),
        "laser_max_range": float(laser_max_range),
        "robot_max_speed": float(robot_max_speed),
    })

def world_to_grid(x: float, y: float) -> Tuple[int,int]:
    x0, y0 = STATE["origin_xy"]
    res = STATE["resolution"]
    r = int((y - y0) / res)
    c = int((x - x0) / res)
    return (r, c)


def _in_bounds(r: int, c: int) -> bool:
    return 0 <= r < STATE["height"] and 0 <= c < STATE["width"]

def _neighbors8(r: int, c: int) -> List[Tuple[int, int]]:
    nbrs = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if _in_bounds(rr, cc):
                nbrs.append((rr, cc))
    return nbrs

def _is_free(v: int) -> bool:
    return v == VAL_FREE

def _is_unknown(v: int) -> bool:
    return v < 0

def _is_occupied(v: int) -> bool:
    return v >= VAL_OCCUPIED


def _grid_to_planner_mask(grid: np.ndarray) -> np.ndarray:
    """
    Binary planning mask: 0 = traversable, 1 = blocked.
    Unknown and occupied are blocked; free is traversable.
    """
    mask = np.ones_like(grid, dtype=np.uint8)
    mask[grid == VAL_FREE] = 0
    return mask

def _astar(mask: np.ndarray, start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
    """
    A* on a binary mask; 0=free, 1=blocked. 8-connected with costs 1 and sqrt(2).
    Returns the path as list of (r,c) from start to goal (inclusive). Empty if none.
    """
    H, W = mask.shape
    sr, sc = start
    gr, gc = goal
    if not (_in_bounds(sr, sc) and _in_bounds(gr, gc)): return []
    if mask[sr, sc] == 1 or mask[gr, gc] == 1: return []

    def h(a: Tuple[int,int], b: Tuple[int,int]) -> float:
        dx = abs(a[1]-b[1]); dy = abs(a[0]-b[0])
        dmin, dmax = min(dx,dy), max(dx,dy)
        return (math.sqrt(2)*dmin + (dmax - dmin))

    gscore = np.full((H, W), np.inf, dtype=float)
    came: Dict[Tuple[int,int], Tuple[int,int]] = {}
    gscore[sr, sc] = 0.0
    openq = [(h((sr,sc),(gr,gc)), 0.0, (sr,sc))]

    while openq:
        _, g, (r, c) = heapq.heappop(openq)
        if (r,c) == (gr,gc):
            path = [(r,c)]
            while (r,c) != (sr,sc):
                r,c = came[(r,c)]
                path.append((r,c))
            path.reverse()
            return path
        for rr, cc in _neighbors8(r, c):
            if mask[rr, cc] == 1: continue
            step = math.sqrt(2.0) if (rr != r and cc != c) else 1.0
            tentative = g + step
            if tentative < gscore[rr, cc]:
                gscore[rr, cc] = tentative
                came[(rr,cc)] = (r,c)
                f = tentative + h((rr,cc), (gr,gc))
                heapq.heappush(openq, (f, tentative, (rr,cc)))
    return []

def _frontiers(grid: np.ndarray) -> List[Set[Tuple[int,int]]]:
    """
    Frontier = free cell adjacent (8-neighborhood) to at least one unknown cell.
    Returns a list of connected components (8-connected) of frontier cells.
    """
    H, W = grid.shape
    frontier_mask = np.zeros((H, W), dtype=bool)
    for r in range(H):
        for c in range(W):
            if not _is_free(grid[r, c]): continue
            for rr, cc in _neighbors8(r, c):
                if _is_unknown(grid[rr, cc]):
                    frontier_mask[r, c] = True
                    break

    visited = np.zeros((H, W), dtype=bool)
    groups: List[Set[Tuple[int,int]]] = []
    for r in range(H):
        for c in range(W):
            if frontier_mask[r, c] and not visited[r, c]:
                comp: Set[Tuple[int,int]] = set()
                stack = [(r, c)]
                visited[r, c] = True
                while stack:
                    cr, cc = stack.pop()
                    comp.add((cr, cc))
                    for nr, nc in _neighbors8(cr, cc):
                        if frontier_mask[nr, nc] and not visited[nr, nc]:
                            visited[nr, nc] = True
                            stack.append((nr, nc))
                groups.append(comp)
    return groups

def _centroid_rc(cells: Set[Tuple[int,int]]) -> Tuple[int,int]:
    rs = [r for r, _ in cells]; cs = [c for _, c in cells]
    r = int(round(sum(rs) / max(1, len(rs))))
    c = int(round(sum(cs) / max(1, len(cs))))
    r = min(max(r, 0), STATE["height"] - 1)
    c = min(max(c, 0), STATE["width"] - 1)
    return (r, c)

def _nearest_free(grid: np.ndarray, rc: Tuple[int,int], max_radius: int = 5) -> Optional[Tuple[int,int]]:
    r0, c0 = rc
    if _in_bounds(r0, c0) and _is_free(grid[r0, c0]): return (r0, c0)
    for rad in range(1, max_radius + 1):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                r, c = r0 + dr, c0 + dc
                if not _in_bounds(r, c): continue
                if abs(dr) != rad and abs(dc) != rad: continue
                if _is_free(grid[r, c]): return (r, c)
    return None

def _cast_ray_unknowns(occ: np.ndarray, start_rc: Tuple[int,int], theta_rad: float, max_range_cells: int) -> Set[Tuple[int,int]]:
    """
    Ray-cast from start_rc at angle theta_rad (radians) up to max_range_cells (in CELLS).
    Adds UNKNOWN cells seen along the ray until hitting an occupied cell, the map boundary,
    or exceeding the distance budget. Unknown is transparent; occupied blocks.
    """
    H, W = occ.shape
    sr, sc = start_rc

    # Position at cell center (grid coords); direction in "cells"
    x = sc + 0.5
    y = sr + 0.5
    dx = math.cos(theta_rad)
    dy = math.sin(theta_rad)

    # Move in fractional cells
    step = 0.25  # cells per integration step

    # Robust distance budget:
    # - ensure int >= 1
    # - clamp to map diagonal to avoid pathological huge ranges from bad resolution values
    max_cells = max(1, int(max_range_cells))
    max_cells = min(max_cells, int(math.hypot(H, W)))

    traveled = 0.0
    seen: Set[Tuple[int,int]] = set()

    while traveled < max_cells:
        x += dx * step
        y += dy * step
        traveled += step

        r = int(y)
        c = int(x)
        if r < 0 or r >= H or c < 0 or c >= W:
            break

        val = occ[r, c]
        # occupied blocks LOS
        if val >= VAL_OCCUPIED:
            break
        # unknown is visible (but does not block)
        if val < 0:
            seen.add((r, c))

    return seen

def _visible_unknowns_best_fov(occ: np.ndarray, at_rc: Tuple[int,int],
                               max_range_cells: int, fov_deg: float,
                               angle_step_deg: float = 5.0) -> Set[Tuple[int,int]]:
    n_angles = max(1, int(round(360.0 / angle_step_deg)))
    base_angles = [math.radians(i * angle_step_deg) for i in range(n_angles)]
    rays: List[Set[Tuple[int,int]]] = [
        _cast_ray_unknowns(occ, at_rc, th, max_range_cells) for th in base_angles
    ]
    k = max(1, int(round(fov_deg / angle_step_deg)))
    rays2 = rays + rays
    best_union: Set[Tuple[int,int]] = set()
    for i in range(n_angles):
        window_union: Set[Tuple[int,int]] = set()
        for j in range(i, i + k):
            window_union |= rays2[j]
        if len(window_union) > len(best_union):
            best_union = window_union
    return best_union

def _line_cells(start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
    """
    Bresenham-like integer line covering both endpoints. Does not check occupancy.
    """
    r0, c0 = start
    r1, c1 = goal
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    s_r = 1 if r0 < r1 else -1
    s_c = 1 if c0 < c1 else -1
    r, c = r0, c0
    cells = [(r, c)]
    if dc > dr:
        err = dc // 2
        while c != c1:
            c += s_c
            err -= dr
            if err < 0:
                r += s_r
                err += dc
            if _in_bounds(r, c):
                cells.append((r, c))
    else:
        err = dr // 2
        while r != r1:
            r += s_r
            err -= dc
            if err < 0:
                c += s_c
                err += dr
            if _in_bounds(r, c):
                cells.append((r, c))
    return cells

# ---- core methods -------------------------------------------------------------
def plan_next_path():
    """
    Plan a multi-goal exploration path up to 10 future goals.
    For each step, choose the goal that maximizes simulated new coverage (unknown cells seen)
    within STATE['robot_fov_deg'] and within STATE['laser_max_range'].
    For steps > 1, the coverage is evaluated on a simulated map where cells revealed by prior
    selected goals are treated as known (set to VAL_FREE). The final path concatenates A* paths
    between successive goals. Overlays are prepared:
      - overlay_goals[r,c] = 20 + i for the i-th future goal (0-indexed),
      - overlay_coverage[r,c] = 30 + i for cells covered from that goal.
    Returns a dict with: future_goals, coverage_sets, final_path, overlay_goals, overlay_coverage
    """
    assert STATE["grid"] is not None, "Call init_from_dump() first."

    grid0: np.ndarray = STATE["grid"]
    res: float = STATE["resolution"]
    fov_deg: float = STATE["robot_fov_deg"]
    max_range_cells: int = max(1, int(STATE["laser_max_range"] / max(res, 1e-9)))

    occ_sim = grid0.copy()
    start_rc: Tuple[int,int] = tuple(STATE["robot_rc"])

    future_goals: List[Tuple[int,int]] = []
    coverage_sets: List[Set[Tuple[int,int]]] = []

    current_rc = _nearest_free(occ_sim, start_rc, max_radius=5) or start_rc
    steps_limit = 10

    for _ in range(steps_limit):
        groups = _frontiers(occ_sim)
        if not groups: break

        best: Optional[Tuple[int,int]] = None
        best_cov: Set[Tuple[int,int]] = set()
        best_score: Tuple[int,int] = (-1, -10**9)  # (coverage_size, -path_len)

        mask = _grid_to_planner_mask(occ_sim)

        for comp in groups:
            cand = _centroid_rc(comp)
            cand = _nearest_free(occ_sim, cand, max_radius=5) or cand
            if not _in_bounds(*cand) or mask[cand[0], cand[1]] == 1:
                continue

            path = _astar(mask, current_rc, cand)
            if not path:  # still consider coverage but strongly penalize unreachable
                cov = _visible_unknowns_best_fov(occ_sim, cand, max_range_cells, fov_deg, angle_step_deg=5.0)
                cov_size = len(cov)
                score = (cov_size, -10**9)  # unreachable -> worst path len
            else:
                cov = _visible_unknowns_best_fov(occ_sim, cand, max_range_cells, fov_deg, angle_step_deg=5.0)
                cov_size = len(cov)
                score = (cov_size, -len(path))

            if score > best_score:
                best_score = score
                best = cand
                best_cov = cov

        if best is None or len(best_cov) == 0: break

        future_goals.append(best)
        coverage_sets.append(best_cov)
        for (rr, cc) in best_cov:
            occ_sim[rr, cc] = VAL_FREE
        current_rc = best

    overlay_goals = np.zeros_like(grid0, dtype=int)
    overlay_cov = np.zeros_like(grid0, dtype=int)
    for i, (goal_rc, cov_set) in enumerate(zip(future_goals, coverage_sets)):
        gr, gc = goal_rc
        overlay_goals[gr, gc] = DUMP_TARGET_BASE + i
        for (rr, cc) in cov_set:
            overlay_cov[rr, cc] = DUMP_VIEW_BASE + i

    # Final path: connect goals; if A* fails for a leg, fall back to straight line.
    final_path: List[Tuple[int,int]] = []
    if future_goals:
        mask0 = _grid_to_planner_mask(grid0)
        cursor = _nearest_free(grid0, start_rc, max_radius=5) or start_rc
        for goal in future_goals:
            path = _astar(mask0, cursor, goal)
            if not path:
                path = _line_cells(cursor, goal)
            final_path += path[1:] if final_path else path
            cursor = goal

    return {
        "future_goals": future_goals,
        "coverage_sets": coverage_sets,
        "final_path": final_path,
        "overlay_goals": overlay_goals,
        "overlay_coverage": overlay_cov,
    }

def plan_path_with_times(start_rc: Tuple[int,int], goal_rc: Tuple[int,int]):
    """
    Plan an A* path from start_rc to goal_rc and estimate travel times.
    Returns: (path_rc, min_time, max_time, real_time)
    - path_rc: list[(r,c)] of visited grid cells
    - min_time: time if robot goes straight at max speed
    - real_time: includes turning penalty
    - max_time: real_time + noise proportional to path length
    """
    assert STATE["grid"] is not None, "Call init_from_dump() first."
    grid: np.ndarray = STATE["grid"]
    res: float = STATE["resolution"]
    vmax: float = max(STATE["robot_max_speed"], 1e-6)

    mask = _grid_to_planner_mask(grid)
    sr, sc = start_rc; gr, gc = goal_rc
    if _in_bounds(sr, sc) and not _is_occupied(grid[sr, sc]): mask[sr, sc] = 0
    if _in_bounds(gr, gc) and not _is_occupied(grid[gr, gc]): mask[gr, gc] = 0

    path: List[Tuple[int,int]] = _astar(mask, start_rc, goal_rc)

    # --- Fallback heuristic: straight line if A* fails ---
    if not path:
        path = _line_cells(start_rc, goal_rc)

    # min_time: straight-line at max speed
    dr = (goal_rc[0] - start_rc[0]); dc = (goal_rc[1] - start_rc[1])
    straight_dist_m = math.hypot(dr, dc) * res
    min_time = straight_dist_m / vmax

    # real_time: along the path + turning penalty
    travel_dist_cells = 0.0
    turn_sum = 0.0
    if len(path) >= 2:
        prev_vec = None
        for i in range(1, len(path)):
            r0, c0 = path[i-1]; r1, c1 = path[i]
            vr = (r1 - r0); vc = (c1 - c0)
            travel_dist_cells += math.hypot(vr, vc)
            cur_angle = math.atan2(vr, vc)
            if prev_vec is not None:
                prev_angle = math.atan2(prev_vec[0], prev_vec[1])
                dtheta = cur_angle - prev_angle
                while dtheta > math.pi: dtheta -= 2 * math.pi
                while dtheta < -math.pi: dtheta += 2 * math.pi
                turn_sum += abs(dtheta)
            prev_vec = (vr, vc)

    path_len_m = travel_dist_cells * res
    travel_time = path_len_m / vmax
    turn_penalty_per_rad = 0.15
    real_time = travel_time + turn_penalty_per_rad * turn_sum
    noise = 0.10 * path_len_m
    max_time = real_time + noise

    return (path, float(min_time), float(max_time), float(real_time))

def updater(json_file_path: str = None):
    parser = argparse.ArgumentParser(description="Update dumped map/odom JSON with time metrics.")
    parser.add_argument("--fov", type=float, default=120.0, help="Robot FOV in degrees.")
    parser.add_argument("--range", dest="laser_range", type=float, default=5.0, help="Laser max range (m).")
    parser.add_argument("--speed", type=float, default=0.4, help="Robot max speed (m/s).")
    args = parser.parse_args()

    # Load input JSON
    with open(json_file_path, "r") as f:
        data = json.load(f)

    # Initialize planner state
    init_from_dump(json_file_path, robot_fov_deg=args.fov,
                   laser_max_range=args.laser_range, robot_max_speed=args.speed)

    # Plan future goals & coverage
    planning = plan_next_path()
    future_goals: List[Tuple[int,int]] = planning["future_goals"]
    coverage_sets: List[Set[Tuple[int,int]]] = planning["coverage_sets"]

    # ---- Time metrics ---------------------------------------------------------
    start_rc = tuple(STATE["robot_rc"])
    path_points: List[Tuple[int,int]] = [start_rc] + future_goals

    _targets = []
    for i in range(len(path_points) - 1):
        start, goal = path_points[i], path_points[i + 1]
        path, tmin, tmax, treal = plan_path_with_times(start, goal)
        _targets.append({
            "start": list(start),
            "goal": list(goal),
            "path_length_m": len(path) * STATE["resolution"],  # match requested metric
            "path_cells": [list(p) for p in path],
            "min_time": tmin,
            "max_time": tmax,
            "real_time": treal
        })

    time_metrics = {
        "num_future_targets": len(future_goals),
        "targets": _targets,
        "total_min_time": sum(t["min_time"] for t in _targets) if _targets else 0.0,
        "total_max_time": sum(t["max_time"] for t in _targets) if _targets else 0.0,
        "total_real_time": sum(t["real_time"] for t in _targets) if _targets else 0.0
    }

    # ---- POV / visibility overlay written into map.data -----------------------
    # Reshape map.data, overlay targets and simulated coverage like your ROS method.
    height = STATE["height"]
    width = STATE["width"]
    dumped_map = np.array(data["map"]["data"], dtype=int).reshape(height, width)

    for i, (trc, vis_set) in enumerate(zip(future_goals[:10], coverage_sets[:10])):
        code_target = DUMP_TARGET_BASE + i  # 20..29
        code_view = DUMP_VIEW_BASE + i      # 30..39
        tr, tc = trc
        if 0 <= tr < height and 0 <= tc < width:
            dumped_map[tr, tc] = code_target
        for (vr, vc) in vis_set:
            if 0 <= vr < height and 0 <= vc < width and dumped_map[vr, vc] == VAL_UNKNOWN:
                dumped_map[vr, vc] = code_view

    data["map"]["data"] = dumped_map.flatten().tolist()
    data["time"] = time_metrics

    # Save JSON
    with open(json_file_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"Updated '{json_file_path}' with time metrics and POV overlays.")


class MyHandler(FileSystemEventHandler):
    def on_any_event(self, event):
        print(event.event_type, event.src_path)

    def on_created(self, event):
        print("on_created", event.src_path)
        sleep(0.5)  # wait for file to be fully written
        if event.src_path.endswith('.json'):
            updater(str(event.src_path))

if __name__ == "__main__":
    event_handler = MyHandler()
    observer = Observer()
    observer.schedule(event_handler, path='./messages/', recursive=False)
    observer.start()

    input('press Enter to quit')

    observer.stop()
