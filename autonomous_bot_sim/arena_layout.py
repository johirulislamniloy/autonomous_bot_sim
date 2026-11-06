import random
from collections import deque

# ---------- Settings (metre) ----------
NUM_OBSTACLES = 8
OBS_SIZE      = 0.08           # square footprint
OBS_HEIGHT    = 0.20
BOT_WIDTH     = 0.14

# Edge-to-edge clearance between any two boxes.  The requirement is >= 0.30 m;
# 0.36 leaves a full robot-width corridor while keeping the scatter free.
MIN_GAP       = 0.36

ARENA_HALF_W  = 0.475          # inner face of the side walls
ARENA_HALF_L  = 1.475          # inner face of the end walls
# Full-width scatter, centre included: the whole arena is sampled so boxes can
# appear in the middle lane instead of only in two fixed side columns.
X_RANGE       = (-0.33, 0.33)
Y_RANGE       = (-1.05, 0.90)  # keep the start and finish zones clear

PLAN_RADIUS   = 0.125          # guaranteed free radius around every path point
GRID          = 0.02
START_X       = 0.0
START_Y       = -1.35
GOAL_Y        = 1.20

OBSTACLE_COLORS = ('red', 'green')


def _box_dist(px, py, o):
    """Edge-to-edge distance from point (px, py) to an axis-aligned box."""
    h = OBS_SIZE / 2.0
    dx = max(abs(px - o[0]) - h, 0.0)
    dy = max(abs(py - o[1]) - h, 0.0)
    return (dx * dx + dy * dy) ** 0.5


def path_exists(obstacles):
    """BFS on a fine grid: can the robot centre reach the finish area?"""
    nx = int(2 * ARENA_HALF_W / GRID)
    ny = int(2 * ARENA_HALF_L / GRID)

    def centre(i, j):
        return (-ARENA_HALF_W + (i + 0.5) * GRID,
                -ARENA_HALF_L + (j + 0.5) * GRID)

    def free(i, j):
        x, y = centre(i, j)
        if abs(x) > ARENA_HALF_W - PLAN_RADIUS:
            return False
        return all(_box_dist(x, y, o) > PLAN_RADIUS for o in obstacles)

    start = (nx // 2, int((START_Y + ARENA_HALF_L) / GRID))
    if not free(*start):
        return False
    goal_j = int((GOAL_Y + ARENA_HALF_L) / GRID)
    seen = {start}
    q = deque([start])
    while q:
        i, j = q.popleft()
        if j >= goal_j:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (i + di, j + dj)
            if 0 <= n[0] < nx and 0 <= n[1] < ny and n not in seen and free(*n):
                seen.add(n)
                q.append(n)
    return False


def pick_positions(rng=random, n=NUM_OBSTACLES):
    """Scatter n boxes uniformly across the whole arena.

    Every box centre is drawn independently from ``X_RANGE`` x ``Y_RANGE`` and
    kept only if its true edge-to-edge distance to every accepted box is at
    least ``MIN_GAP``.  Because the sampling is uniform over the full width,
    layouts are genuinely different each run and boxes land in the middle lane
    as often as near the walls (unlike a fixed 2-column grid).

    A BFS check then guarantees that at least one robot-width corridor still
    connects the start to the finish.
    """
    for _ in range(3000):
        pts = []
        ok = True
        for _ in range(n):
            for _ in range(400):
                x = rng.uniform(*X_RANGE)
                y = rng.uniform(*Y_RANGE)
                if all(_box_dist(x, y, p) >= MIN_GAP for p in pts):
                    pts.append((x, y))
                    break
            else:
                ok = False
                break
        if ok and path_exists(pts):
            return pts

    # Very unlikely fallback: staggered three-column pattern with valid gaps.
    pts = []
    for x, rows in ((-0.28, (-0.95, -0.05, 0.85)),
                    (0.00, (-0.50, 0.45)),
                    (0.28, (-0.95, -0.05, 0.85))):
        for y in rows:
            pts.append((x, y))
    if path_exists(pts):
        return pts
    raise RuntimeError('Could not find a valid scattered layout')


def pick_colors(rng=random, n=NUM_OBSTACLES):
    """Half red, half green, shuffled every run."""
    colors = ['red'] * (n // 2) + ['green'] * (n - n // 2)
    rng.shuffle(colors)
    return colors
