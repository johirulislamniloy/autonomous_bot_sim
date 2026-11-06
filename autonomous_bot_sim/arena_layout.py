import random
from collections import deque

# ---------- Settings (meter) ----------
NUM_OBSTACLES = 10
OBS_SIZE      = 0.10
OBS_HEIGHT    = 0.20
BOT_WIDTH     = 0.14
MIN_GAP       = 0.25          # only used for the log message now

ARENA_HALF_W  = 0.475
ARENA_HALF_L  = 1.475
# Keep obstacles inside a central band so EITHER colour rule (red -> pass
# left, green -> pass right) always has enough room next to the side wall.
X_RANGE       = (-0.20, 0.20)
Y_RANGE       = (-1.10, 1.10)

MIN_DIST      = 0.27          # centre-to-centre (Euclidean) between obstacles
PLAN_RADIUS   = 0.085         # bot half-width (0.07) + margin
GRID          = 0.02

# Half of the obstacles are red, half are green.  The order is shuffled so
# both colours appear at random places every run.
OBSTACLE_COLORS = ('red', 'green')


def far_enough(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 >= MIN_DIST ** 2


def _box_dist(px, py, o):
    h = OBS_SIZE / 2.0
    dx = max(abs(px - o[0]) - h, 0.0)
    dy = max(abs(py - o[1]) - h, 0.0)
    return (dx * dx + dy * dy) ** 0.5


def path_exists(obstacles):
    nx = int(2 * ARENA_HALF_W / GRID)
    ny = int(2 * ARENA_HALF_L / GRID)

    def centre(i, j):
        return (-ARENA_HALF_W + (i + 0.5) * GRID, -ARENA_HALF_L + (j + 0.5) * GRID)

    def free(i, j):
        x, y = centre(i, j)
        if abs(x) > ARENA_HALF_W - PLAN_RADIUS:
            return False
        return all(_box_dist(x, y, o) > PLAN_RADIUS for o in obstacles)

    start = (nx // 2, int((-1.35 + ARENA_HALF_L) / GRID))
    if not free(*start):
        return False
    seen = {start}
    q = deque([start])
    goal_j = int((1.20 + ARENA_HALF_L) / GRID)
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


def pick_positions(rng=random, n=NUM_OBSTACLES, colors=None):
    """Place the boxes as compact colour gates: two boxes of one colour per row.

    The pair sits on the CLOSED side of the lane, close to the centre line:
    the inner box is almost dead ahead (so the single TOF ray can range it)
    and the outer box is only 0.15 m further to that same side.  A red gate
    therefore leaves a wide lane on the robot's LEFT and a green gate on its
    RIGHT.  Because the pair is compact, the sidestep is only ~0.15 m and the
    robot can re-centre in the gap before the next row.

    Rows are spaced ~0.57 m apart, which is what the stop -> turn -> sidestep
    -> cross -> re-centre cycle needs in a 3 m arena.
    """
    gates = n // 2
    y0, y1 = -1.10, 1.18
    step = (y1 - y0) / max(1, gates - 1)
    for _ in range(200):
        pts = []
        for g in range(gates):
            y = y0 + g * step
            color = colors[2 * g] if colors else None
            sign = 1.0 if color != 'green' else -1.0   # red pair on the robot's right
            inner = rng.uniform(-0.008, 0.008)          # dead ahead: easy to see
            outer = sign * 0.150 + rng.uniform(-0.010, 0.010)
            pts.append((inner, y + rng.uniform(-0.010, 0.010)))
            pts.append((outer, y + rng.uniform(-0.010, 0.010)))
        # any leftover slots (odd n) go on the open side of the last gate
        while len(pts) < n:
            g = len(pts) // 2
            y = y0 + min(g, gates - 1) * step
            pts.append((0.0, y + 0.04 * (len(pts) % 3)))
        if path_exists(pts):
            return pts
    raise RuntimeError('Could not find a valid obstacle layout')


def pick_colors(rng=random, n=NUM_OBSTACLES):
    """Exactly half red / half green, as same-colour gates.

    Each gate is two boxes of one colour and consecutive gates alternate, so
    the robot has to swing left, then right, then left ... exactly the
    colour rule.  The first gate colour is still random every run.
    """
    first = rng.choice(OBSTACLE_COLORS)
    second = 'green' if first == 'red' else 'red'
    return [first if (i // 2) % 2 == 0 else second for i in range(n)]

