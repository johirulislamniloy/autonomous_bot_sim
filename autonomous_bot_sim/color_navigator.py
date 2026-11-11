"""Multi-mode obstacle-course navigator (pure python, no ROS -> unit-testable).

Pick the algorithm with the `mode` launch/ROS parameter (or nav.set_mode()):

    astar   Map-aware A* route over the known box layout, followed with pure
            pursuit.  If the layout is unknown / no path exists it falls back to
            the reactive scan-and-go below, then plans again.

    scan    Hybrid Bug2 + scan-and-go (reactive, no map needed):

              SCAN      stand still, sweep servo+TOF once, pick the most open
                        bearing (longest TOF corridor, biased to the finish)
              TURN_GO   rotate in place onto that bearing (servo stays straight)
              APPROACH  drive straight along the bearing, stop DECIDE_DIST
                        before a box (or after the free run length)
              DECIDE    target was LEFT  -> turn RIGHT (toward the arena middle)
                        target was RIGHT -> turn LEFT
                        target STRAIGHT  -> stop, SCAN again (limited), then
                                            pass on the more open side
              TURN_SIDE / SIDE_MOVE   diagonal detour past the box
              TURN_BACK / CROSS       turn back to the finish heading and go a
                                      bit forward (body straight), then SCAN
              ESCAPE    no progress (front TOF flat, side sonar scraping, or
                        IMU yaw flat during a turn) -> reverse while probing
                        for room, STRAIGHTEN, then resume the cycle

    bug2    Classic Bug2 with only a straight TOF + 2 side sonars: drive along
            the M-line (lateral 0), on a hit turn away, wall-follow with the side
            sonar, leave when back on the M-line and closer to the goal.

    open    Competition open round: follow the outer wall of the 1 m wide
            square ring, turn 90 deg at each corner on the IMU, and stop after
            3 complete laps (24 m).  The driving direction (clockwise or
            counter-clockwise) is assigned randomly at the event.

Shared by all obstacle modes: FINAL (cyan strip seen / finish zone -> dash).

Angles are relative to the start heading: 0 = toward the finish, + = left.
Box colour is never used.
"""

import heapq
import math

from autonomous_bot_sim.arena_layout import OBS_SIZE, PLAN_RADIUS, ARENA_HALF_W

INF = 9.9
MODES = ('astar', 'scan', 'bug2', 'open')
_ALIASES = {'a*': 'astar', 'a_star': 'astar', 'a-star': 'astar',
            'planner': 'astar', 'map': 'astar',
            'hybrid': 'scan', 'scan_and_go': 'scan', 'scan-and-go': 'scan',
            'scango': 'scan', 'bug': 'bug2', 'bug_2': 'bug2',
            'open_round': 'open', 'open-round': 'open', 'ring': 'open',
            'square': 'open', 'openround': 'open'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ColorCourseNavigator:
    # ---------------------------------------------------------------- arena
    WALL_P        = 2.825       # end wall, measured from the start pose
    WALL_SIDE     = 0.475       # side wall, measured from the lane centre
    WALL_STOP     = 0.075       # raw TOF distance at which the wall guard stops
    BEAM_X        = 0.100       # TOF pivot ahead of the axle
    SONAR_Y       = 0.075       # side sonar offset from the centre line
    FINISH_P      = 2.525       # axle progress on the cyan strip
    LINE_MIN_P    = 2.175       # closest sensible start of the finish zone

    # ---------------------------------------------------------------- scan
    SWEEP_AMP     = 1.05
    BIN           = 0.05
    SCAN_SETTLE   = 0.30
    SCAN_SWEEP    = 1.90
    SCAN_RETURN   = 0.30
    SCAN_TOTAL    = SCAN_SETTLE + SCAN_SWEEP + SCAN_RETURN
    MAX_RANGE     = 3.0
    SCAN_BIAS     = 0.38        # metre penalty per rad away from the finish
    SCAN_WINDOW   = 0.30        # rad: corridor half-width used to reject gaps
    SCAN_FWD_LIMIT = math.radians(70)   # never aim outside this finish sector

    # -------------------------------------------------------------- turning
    K_TURN        = 2.2
    TURN_W_MAX    = 0.90
    TURN_W_MIN    = 0.22
    ALIGN_TOL     = math.radians(3)
    SETTLE        = 0.18

    # -------------------------------------------------------------- driving
    V_FWD         = 0.125
    V_PASS        = 0.080
    V_FINAL       = 0.070
    K_HEAD        = 2.8
    W_MAX         = 0.85

    # ------------------------------------------- scan-and-go: stop / decide
    DECIDE_DIST   = 0.160       # stop this far before a box (room to turn)
    STRAIGHT_TOL  = math.radians(12)   # target inside this -> "straight"
    STRAIGHT_RESCANS = 1        # re-scans before a straight box is forced past
    OVERRIDE_BEARING = 0.15     # rad: box clearly off-axis -> pass away from it
    MIN_RUN       = 0.06        # a scan run shorter than this is "no room"
    MAX_RUN       = 1.00        # longest single run between scans
    LANE_LIM      = WALL_SIDE - 0.150   # keep a run inside this lateral limit

    # Diagonal detour used to pass a box (Bug2 style).
    DETOUR_ANGLE  = math.radians(62)
    DETOUR_DIST   = 0.320
    DETOUR_MIN_MOVE = 0.100

    # Scan memory: boxes seen during the sweep that a straight ray would miss.
    MEM_RADIUS    = 0.020
    MEM_DEDUP     = 0.060
    MEM_MAX_PTS   = 400
    MEM_RANGE     = 0.900

    # ---------------------------------------------------------- side sonars
    SIDE_GUARD    = 0.160       # gentle push away from anything this close
    SIDE_STOP     = 0.015
    SIDE_HARD     = 0.100
    SIDE_REVERSE  = 0.035       # box scraping the body
    PASS_CLEAR_MIN = 0.170
    SIDE_CLEAR_BOX = 0.180
    PASS_LAT_MAX  = 0.260

    # ------------------------------------------------------------- crossing
    CROSS_MIN     = 0.280
    CROSS_MAX_T   = 5.0

    # ---------------------------------------------------------- map planner
    OBS_HALF      = OBS_SIZE / 2.0
    PLAN_DP       = 0.02
    PLAN_P_MIN    = -0.05
    PLAN_P_MAX    = 2.72
    PLAN_L_MAX    = ARENA_HALF_W - PLAN_RADIUS
    PLAN_CLEAR    = PLAN_RADIUS
    PLAN_GOAL_P   = 2.45
    WP_TOL        = 0.070
    LOOKAHEAD     = 0.200

    # ----------------------------------------------------------------- bug2
    BUG_HIT_FRONT = 0.15        # straight TOF distance that counts as a hit
    BUG_HIT_SIDE  = 0.05        # side sonar on a box -> hit
    K_LAT         = 1.5
    LAT_HEAD_MAX  = 0.30
    ALIGN_W       = 0.7
    ALIGN_MIN     = 0.5
    ALIGN_MAX     = 1.45
    FOLLOW_SEE    = 0.20
    CLEAR_F       = 0.20
    V_FOLLOW      = 0.10
    D_FOLLOW      = 0.08
    K_WALL        = 5.0
    LOST          = 0.22
    PASS_STRAIGHT = 0.06
    V_CORNER      = 0.08
    W_CORNER      = 0.6
    CORNER_TURN_W = 0.8
    INNER_FRONT   = 0.12
    LOST_LEAVE    = 0.40
    OFF_LINE      = 0.07
    M_TOL         = 0.05
    GAIN          = 0.02
    BUG_LANE_HALF = 0.32
    T_FOLLOW      = 25.0
    BUG_WIGGLE    = 0.45        # servo dither (rad) while driving in bug2 mode
    BUG_WIGGLE_T  = 0.8         # dither period (s): the TOF must cover the body width

    # ------------------------------------------------------- open round ring
    # 3 x 3 m outer square, 1 x 1 m inner block -> 1 m wide track.  The robot
    # follows the 2 x 2 m centreline: 8 m per lap, 24 m for three laps.
    OPEN_LAPS        = 3
    OPEN_SIDE_LEN    = 2.0
    OPEN_LAP_LEN     = 8.0
    OPEN_TOTAL       = OPEN_LAPS * OPEN_LAP_LEN
    OPEN_V           = 0.130
    OPEN_SIDE_TARGET = 0.425  # sonar reading to either wall on the centreline
    OPEN_K_OUTER     = 3.0
    OPEN_INNER_SAFE  = 0.255  # closer than this to the inner block -> steer out
    OPEN_K_INNER     = 3.2
    OPEN_CORNER_FRONT = 0.48  # TOF range from the corner wall -> 90 deg turn
    OPEN_CORNER_TIMEOUT = 5.0
    OPEN_STALL_T     = 1.5
    OPEN_STALL_EPS   = 0.010
    OPEN_STALL_R     = 0.30
    OPEN_STOP_TOL    = 0.025  # m, stop when the front TOF matches the start
    OPEN_STOP_HITS   = 3      # consecutive matched samples (20 Hz loop)
    OPEN_STOP_EXTRA_T = 0.30  # s, last-resort guard past the clock estimate

    # ---------------------------------------------------------- recovery
    # Stuck: motor commanded forward but the robot makes no progress.  With no
    # wheel encoders this is detected purely from the sensors:
    #   * front TOF range flat while driving forward,
    #   * side sonar pinned very close and flat while driving forward,
    #   * IMU yaw flat while a turn is commanded.
    TOF_STUCK_T   = 1.5
    TOF_STUCK_EPS = 0.015
    TOF_WATCH_R   = 0.14        # only a close, unchanging return means stalled;
                                # a box at arm's length can keep a constant
                                # range while the robot manoeuvres near it
    # Bug2's straight M-line drive has almost no yaw, so a flat *raw* front
    # return (walls included) is a reliable no-progress signal there.  Other
    # states arc around boxes and can hold a constant raw range while moving.
    GO_RAW_STUCK_T = 1.5
    GO_RAW_WATCH_R = 1.50
    SIDE_STUCK_T  = 1.5
    SIDE_STUCK_EPS = 0.010
    SIDE_STUCK_R  = 0.030       # "pressed against something" range
    TURN_STUCK_T  = 1.0
    TURN_STUCK_EPS = math.radians(6)
    # Escape manoeuvre: reverse while probing for room, then straighten.
    ESCAPE_T      = 1.4         # nominal reverse time
    ESCAPE_MAX_T  = 2.4         # hard stop even if still blocked
    ESCAPE_MIN_T  = 0.50        # never leave before this
    ESCAPE_FRONT_CLEAR = 0.28   # front TOF distance that counts as room
    ESCAPE_SIDE_CLEAR  = 0.10   # both side sonars must be clearer than this
    T_REVERSE     = 0.75        # grows 50 % per consecutive reverse (max x2.5)
    V_REVERSE     = 0.060
    T_STRAIGHTEN  = 2.0

    # -------------------------------------------------------------- timeouts
    T_APPROACH    = 12.0
    T_TURN        = 4.0
    T_SIDE        = 6.0
    T_FINAL       = 30.0        # a reactive mode can enter FINAL early and
                                # off-centre; allow time to straighten, ease
                                # back to the middle and creep to the strip

    DEBUG         = True

    def __init__(self, mode='astar', obs_size=None, finish_y=None,
                 open_clockwise=False):
        self.obstacles = []
        self.mode = mode if mode in MODES else 'astar'
        self.open_clockwise = bool(open_clockwise)
        # Obstacle-round finish target: the launch preset passes the
        # competition finish_y; None keeps the built-in value.
        self.obs_half = (OBS_SIZE if obs_size is None
                         else float(obs_size)) / 2.0
        if finish_y is None:
            self.finish_p = self.FINISH_P
            self.line_min_p = self.LINE_MIN_P
        else:
            self.finish_p = float(finish_y) + 1.35
            self.line_min_p = self.finish_p - (self.FINISH_P - self.LINE_MIN_P)
        self.reset(0.0, 0.0, 0.0)

    # ------------------------------------------------------------ mode switch
    def set_mode(self, mode):
        """Select 'astar', 'scan' or 'bug2'.  Safe to call while running: the
        current manoeuvre is dropped and the new algorithm starts from the next
        control tick (the FINAL dash is never interrupted)."""
        key = str(mode).strip().lower()
        key = _ALIASES.get(key, key)
        if key not in MODES:
            print('[nav] unknown mode %r (use %s) -> keeping %s'
                  % (mode, '/'.join(MODES), self.mode), flush=True)
            return False
        if key != self.mode:
            self.mode = key
            self.path = []
            if self.state not in (None, 'FINAL') and not self.finished:
                self.scan_active = False
                self.state = None
            print('[nav] mode -> %s' % key, flush=True)
        return True

    def set_planner_enabled(self, enabled):
        """Backwards compatible with the old `planner` bool parameter."""
        self.set_mode('astar' if enabled else 'scan')

    @property
    def planner_enabled(self):
        return self.mode == 'astar'

    def set_open_clockwise(self, clockwise):
        """Direction for the open round: True = clockwise, False = CCW.

        Safe to call before or during a run.  The controller takes it from
        /open_clockwise in simulation, or from the ``open_clockwise`` ROS
        parameter on the real robot.
        """
        self.open_clockwise = bool(clockwise)
        if self.mode == 'open':
            print('[nav] open round direction: %s'
                  % ('clockwise' if self.open_clockwise
                     else 'counter-clockwise'), flush=True)

    # ------------------------------------------------------------------ init
    def reset(self, x0, y0, yaw0):
        self.x0, self.y0, self.yaw0 = x0, y0, yaw0
        self.state = None
        self.t_state = 0.0
        self.finished = False
        self.finish_reason = None
        self.finish_seen = False
        self.last_finish_t = -1e9
        self.maneuvers = 0

        self.x, self.y, self.rel = x0, y0, 0.0
        self.target_abs = 0.0
        self.scan_target_rel = 0.0
        self.run_len = 0.0
        self.run_box = False
        self.straight_scans = 0
        self.decide_ref = None
        self.pass_heading = 0.0
        self.pass_side = 0
        self.side_need = self.PASS_CLEAR_MIN
        self.box_rel = 0.0
        self.blocked_side = 0       # side whose detour was just blocked

        self.scan = {}
        self.scan_t0 = None
        self.scan_active = False
        self.mem_pts = []
        self.recent = []
        self.path = []
        self.path_i = 0

        # bug2
        self.obs_side = +1
        self.hit_p = self.hit_rel = self.hit_lat = 0.0
        self.hits = 0
        self.off_line = False
        self.lost_xy = None
        self.uturn_target = 0.0

        # recovery
        self.tof_anchor = None
        self.tof_anchor_t = 0.0
        self.raw_tof_anchor = None
        self.raw_tof_anchor_t = 0.0
        self.side_anchor = None
        self.side_anchor_t = 0.0
        self.rel_anchor = None
        self.rel_anchor_t = 0.0
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0
        self.reverse_count = 0
        self.front_d = INF
        self.raw_d = INF
        self.sx = self.sy = 0.0
        self.lat_entry = 0.0
        self.ok_since = None
        self._dbg = -1

        # open round
        self.open_heading = 0.0     # target tangent, relative to start heading
        self.open_dist = 0.0        # dead-reckoned centreline distance
        self.open_corners = 0
        self.open_first_leg = None  # distance from start to the first corner
        self.open_finish_from = None
        self.open_finish_remaining = None
        self.open_prev_x = x0
        self.open_prev_y = y0
        self.open_stall_anchor = None
        self.open_stall_t = 0.0

        # Open-round start memory, clock only (no encoders, no line):
        #   A = time from the start to the first corner (partial side)
        #   B = average time of one full side, measured over the 11 sides
        #       between corners 1..12
        # After the 12th corner the robot is on the starting side and drives
        # for (B - A) seconds: (full side) - (partial first leg), i.e. exactly
        # back to the remembered start point.
        self.open_start_t = None
        self.open_last_turn_done_t = None
        self.open_corner_intervals = []
        self.open_first_leg_t = None    # A
        self.open_side_time = None      # B
        self.open_final_leg_t = None    # B - A (clock fallback / guard)
        self.open_final_leg_t0 = None   # when the final leg began
        # Primary stop reference: the front TOF at the start point.  After
        # three laps the robot is on the same side heading the same way, so
        # the same end wall gives the same forward range: an absolute
        # along-track position with no encoder or clock drift.
        self.open_start_tof = None
        self.open_tof_hits = 0

    # ---------------------------------------------------------- pose helpers
    def progress(self, x, y):
        return ((x - self.x0) * math.cos(self.yaw0)
                + (y - self.y0) * math.sin(self.yaw0))

    def lateral(self, x, y):
        """Signed distance from the start line: + = left of the start heading."""
        return (-(x - self.x0) * math.sin(self.yaw0)
                + (y - self.y0) * math.cos(self.yaw0))

    # ------------------------------------------------------------ map planner
    def set_obstacles(self, points):
        """Box centres in the navigator (progress, lateral) frame."""
        self.obstacles = [(float(p), float(l)) for p, l in points]
        self.path = []
        self.path_i = 0
        if self.obstacles:
            print('[nav] map: %d obstacle centres received'
                  % len(self.obstacles), flush=True)

    def _clearance(self, p, l):
        best = INF
        for op, ol in self.obstacles:
            dp = max(0.0, abs(p - op) - self.obs_half)
            dl = max(0.0, abs(l - ol) - self.obs_half)
            d = math.hypot(dp, dl)
            if d < best:
                best = d
        return best

    def _to_cell(self, p, l):
        return (int(round((p - self.PLAN_P_MIN) / self.PLAN_DP)),
                int(round((l + self.PLAN_L_MAX) / self.PLAN_DP)))

    def _from_cell(self, i, j):
        return (self.PLAN_P_MIN + i * self.PLAN_DP,
                -self.PLAN_L_MAX + j * self.PLAN_DP)

    def _plan_path(self):
        """A* over the known layout, returned as a smoothed waypoint list."""
        if not self.obstacles:
            return None
        dp = self.PLAN_DP
        ni = int(round((self.PLAN_P_MAX - self.PLAN_P_MIN) / dp)) + 1
        nj = int(round(2.0 * self.PLAN_L_MAX / dp)) + 1

        clear = [[0.0] * nj for _ in range(ni)]
        for i in range(ni):
            p = self.PLAN_P_MIN + i * dp
            row = clear[i]
            for j in range(nj):
                row[j] = self._clearance(p, -self.PLAN_L_MAX + j * dp)

        def free(i, j):
            return 0 <= i < ni and 0 <= j < nj \
                and clear[i][j] >= self.PLAN_CLEAR - 1e-6

        si, sj = self._to_cell(self.x, self.y)
        si = min(max(si, 0), ni - 1)
        sj = min(max(sj, 0), nj - 1)
        if not free(si, sj):
            found = None
            for rad in range(1, 12):
                for di in range(-rad, rad + 1):
                    for dj in range(-rad, rad + 1):
                        if max(abs(di), abs(dj)) != rad:
                            continue
                        if free(si + di, sj + dj):
                            found = (si + di, sj + dj)
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                return None
            si, sj = found

        gi, gj = self._to_cell(self.PLAN_GOAL_P, 0.0)
        gi = min(max(gi, 0), ni - 1)
        gj = min(max(gj, 0), nj - 1)
        if not free(gi, gj):
            found = None
            for rad in range(1, 12):
                for dj in range(0, nj):
                    if free(gi, dj) and abs(dj - gj) <= rad:
                        found = (gi, dj)
                        break
                if found:
                    break
            if not found:
                return None
            gi, gj = found

        def h(i, j):
            return math.hypot((i - gi) * dp, (j - gj) * dp)

        gscore = [[INF] * nj for _ in range(ni)]
        prev = {}
        gscore[si][sj] = 0.0
        openq = [(h(si, sj), 0.0, si, sj)]
        neigh = ((1, 0), (-1, 0), (0, 1), (0, -1),
                 (1, 1), (1, -1), (-1, 1), (-1, -1))
        while openq:
            _, g, i, j = heapq.heappop(openq)
            if (i, j) == (gi, gj):
                break
            if g > gscore[i][j] + 1e-9:
                continue
            for di, dj in neigh:
                ni_, nj_ = i + di, j + dj
                if not free(ni_, nj_):
                    continue
                if di and dj and not (free(i + di, j) and free(i, j + dj)):
                    continue
                step = math.hypot(di, dj) * dp
                penalty = 8.0 * max(0.0, 0.32 - clear[ni_][nj_])
                ng = g + step * (1.0 + penalty)
                if ng + 1e-9 < gscore[ni_][nj_]:
                    gscore[ni_][nj_] = ng
                    prev[(ni_, nj_)] = (i, j)
                    heapq.heappush(openq, (ng + h(ni_, nj_), ng, ni_, nj_))

        if gscore[gi][gj] >= INF:
            return None

        cells = [(gi, gj)]
        cur = (gi, gj)
        while cur != (si, sj):
            cur = prev.get(cur)
            if cur is None:
                return None
            cells.append(cur)
        cells.reverse()
        raw = [self._from_cell(i, j) for i, j in cells]
        return self._smooth_path(raw)

    def _segment_clear(self, a, b):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(dist / (self.PLAN_DP * 0.5)))
        for k in range(n + 1):
            f = k / n
            p = a[0] + (b[0] - a[0]) * f
            l = a[1] + (b[1] - a[1]) * f
            if abs(l) > self.PLAN_L_MAX + 1e-6:
                return False
            if self._clearance(p, l) < self.PLAN_CLEAR - 0.005:
                return False
        return True

    def _smooth_path(self, raw):
        if len(raw) <= 2:
            return raw
        out = [raw[0]]
        anchor = 0
        while anchor < len(raw) - 1:
            far = anchor + 1
            for k in range(len(raw) - 1, anchor, -1):
                if self._segment_clear(raw[anchor], raw[k]):
                    far = k
                    break
            out.append(raw[far])
            anchor = far
        return out

    def _path_target(self, x, y):
        """Pure-pursuit point LOOKAHEAD metres ahead of the projection of the
        robot on the path.  Returns (point, segment_index)."""
        path = self.path
        if len(path) < 2:
            return (path[0] if path else (x, y)), 0
        best = (INF, 0, 0.0)
        for k in range(len(path) - 1):
            ax, ay = path[k]
            bx, by = path[k + 1]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            if l2 < 1e-12:
                u = 0.0
            else:
                u = clamp(((x - ax) * dx + (y - ay) * dy) / l2, 0.0, 1.0)
            d = math.hypot(x - (ax + u * dx), y - (ay + u * dy))
            if d < best[0]:
                best = (d, k, u)
        _, k, u = best

        remaining = self.LOOKAHEAD
        ax, ay = path[k]
        bx, by = path[k + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg > 1e-9:
            avail = (1.0 - u) * seg
            if remaining <= avail:
                f = u + remaining / seg
                return (ax + (bx - ax) * f, ay + (by - ay) * f), k
            remaining -= avail
        for kk in range(k + 1, len(path) - 1):
            ax, ay = path[kk]
            bx, by = path[kk + 1]
            seg = math.hypot(bx - ax, by - ay)
            if seg <= 1e-9:
                continue
            if remaining <= seg:
                f = remaining / seg
                return (ax + (bx - ax) * f, ay + (by - ay) * f), kk
            remaining -= seg
        return path[-1], len(path) - 1

    # ------------------------------------------------------------- servo/TOF
    def servo_cmd(self, t):
        """Sweep only while scanning; hold dead straight while driving."""
        if not self.scan_active:
            if self.mode == 'scan' and self.state == 'SIDE_MOVE':
                # The box being passed sits well off the straight beam; a
                # short dither lets the front TOF see it during the sidestep.
                return 0.40 * math.sin(2.0 * math.pi * t / 0.45)
            if self.mode == 'bug2' and self.state in ('GO', 'B_LEAVE'):
                # A single straight ray misses a box edge inside the 14 cm
                # body width, so Bug2 dithers the beam while driving.  Keep
                # dithering in B_LEAVE too: the realign turn lasts over a
                # second, which would otherwise let every off-axis sample
                # expire and hide the box that is still beside the body.
                return self.BUG_WIGGLE * math.sin(
                    2.0 * math.pi * t / self.BUG_WIGGLE_T)
            return 0.0
        e = t - self.scan_t0
        if e < self.SCAN_SETTLE:
            return -self.SWEEP_AMP
        e -= self.SCAN_SETTLE
        if e < self.SCAN_SWEEP:
            return -self.SWEEP_AMP + 2.0 * self.SWEEP_AMP * (e / self.SCAN_SWEEP)
        e -= self.SCAN_SWEEP
        if e < self.SCAN_RETURN:
            return self.SWEEP_AMP * (1.0 - e / self.SCAN_RETURN)
        self.scan_active = False
        return 0.0

    def tof(self, bearing, rng, t):
        """Feed one TOF sample: servo joint angle (rad) and range (m)."""
        if not math.isfinite(rng):
            return
        self.recent.append((t, wrap(self.rel + bearing), rng))
        if len(self.recent) > 160:
            del self.recent[:-160]
        if self.scan_active and self.scan_t0 is not None \
                and t - self.scan_t0 <= self.SCAN_TOTAL + 0.10 \
                and abs(bearing) <= self.SWEEP_AMP + 0.06:
            b = int(round(bearing / self.BIN))
            if b not in self.scan or rng < self.scan[b]:
                self.scan[b] = rng

    # ------------------------------------------------------------- wall tests
    def _is_wall(self, d, bearing):
        c = math.cos(self.rel + bearing)
        if c < 0.3:
            return False
        p_b = self.progress(self.x, self.y) + self.BEAM_X * math.cos(self.rel)
        expected = (self.WALL_P - p_b) / c
        if expected > 1.2:
            return False
        return d >= expected - 0.18

    def _is_side_wall(self, d, bearing):
        a = self.rel + bearing
        c_l = math.sin(a)
        if abs(c_l) < 0.20:
            return False
        lat_b = self.lateral(self.x, self.y) + self.BEAM_X * math.sin(self.rel)
        wall = self.WALL_SIDE if c_l > 0 else -self.WALL_SIDE
        expected = (wall - lat_b) / c_l
        if expected <= 0.0 or expected > 1.4:
            return False
        return d >= expected - 0.15

    def _front(self, t, cone=0.95):
        """Nearest fresh TOF return inside the forward cone, walls removed.
        Returns (bearing, range, forward_distance) or None."""
        vals = []
        for tt, ab, r in reversed(self.recent):
            if t - tt > 0.90:
                break
            a = wrap(ab - self.rel)
            if abs(a) <= cone and not self._is_wall(r, a) \
                    and not self._is_side_wall(r, a):
                vals.append((a, r))
            if len(vals) >= 24:
                break
        if not vals:
            return None
        a, r = min(vals, key=lambda v: v[1] * math.cos(v[0]))
        return a, r, r * math.cos(a)

    def _raw_front(self, t, cone=0.40):
        """Nearest fresh TOF return including walls."""
        vals = []
        for tt, ab, r in reversed(self.recent):
            if t - tt > 0.90:
                break
            a = wrap(ab - self.rel)
            if abs(a) <= cone:
                vals.append((a, r))
            if len(vals) >= 24:
                break
        if not vals:
            return None
        a, r = min(vals, key=lambda v: v[1] * math.cos(v[0]))
        return a, r, r * math.cos(a)

    def _remember_scan(self, rel):
        """Store non-wall SCAN hits as fixed points in the navigator frame."""
        for b, r in self.scan.items():
            if not math.isfinite(r) or r >= self.MEM_RANGE:
                continue
            a = b * self.BIN
            if self._is_wall(r, a) or self._is_side_wall(r, a):
                continue
            ang = rel + a
            px = self.x + r * math.cos(ang)
            py = self.y + r * math.sin(ang)
            if any((px - qx) ** 2 + (py - qy) ** 2
                   < self.MEM_DEDUP ** 2 for qx, qy in self.mem_pts):
                continue
            self.mem_pts.append((px, py))
            if len(self.mem_pts) > self.MEM_MAX_PTS:
                del self.mem_pts[0]

    def _memory_obstacle(self, half_width, max_r):
        best = None
        c, s = math.cos(self.rel), math.sin(self.rel)
        for qx, qy in self.mem_pts:
            dx, dy = qx - self.x, qy - self.y
            fwd = dx * c + dy * s
            if not (self.MEM_RADIUS < fwd < max_r):
                continue
            lat = -dx * s + dy * c
            if abs(lat) >= half_width + self.MEM_RADIUS:
                continue
            d = fwd - self.MEM_RADIUS
            if best is None or d < best[2]:
                best = (math.atan2(lat, fwd), math.hypot(dx, dy), d)
        return best

    def _path_obstacle(self, t, half_width=0.14, max_r=1.00, cone=0.95):
        """Nearest box that would hit the body (live TOF + scan memory).
        Returns (bearing, range, forward_distance) or None."""
        best = self._memory_obstacle(half_width, max_r)
        for tt, ab, r in reversed(self.recent):
            if t - tt > 0.90:
                break
            a = wrap(ab - self.rel)
            if abs(a) > cone:
                continue
            if self._is_wall(r, a) or self._is_side_wall(r, a):
                continue
            fwd = r * math.cos(a)
            lat = r * math.sin(a)
            if 0.0 < fwd < max_r and abs(lat) < half_width:
                if best is None or fwd < best[2]:
                    best = (a, r, fwd)
        return best

    def _side_box(self, sl, sr, lat, thresh=0.30):
        """+1 if the LEFT sonar sees a box (not the wall), -1 for RIGHT, else 0."""
        exp_l = self.WALL_SIDE - lat - self.SONAR_Y
        exp_r = self.WALL_SIDE + lat - self.SONAR_Y
        box_l = sl < min(thresh, exp_l - 0.08)
        box_r = sr < min(thresh, exp_r - 0.08)
        if box_l and box_r:
            return +1 if sl <= sr else -1
        if box_l:
            return +1
        if box_r:
            return -1
        return 0

    # --------------------------------------------------------- stuck detection
    def _tof_stuck(self, t, dist, moving):
        """True when forward motion is commanded but the TOF never changes."""
        if (not moving or not math.isfinite(dist)
                or dist > self.TOF_WATCH_R):
            self.tof_anchor = None
            return False
        if self.tof_anchor is None:
            self.tof_anchor = dist
            self.tof_anchor_t = t
            return False
        if abs(dist - self.tof_anchor) > self.TOF_STUCK_EPS:
            self.tof_anchor = dist
            self.tof_anchor_t = t
            return False
        return (t - self.tof_anchor_t) >= self.TOF_STUCK_T

    def _raw_tof_stuck(self, t):
        """Legacy raw-front stall check, used only in Bug2's GO state."""
        dist = self.raw_d
        if (self.mode != 'bug2' or self.state != 'GO'
                or not math.isfinite(dist) or dist > self.GO_RAW_WATCH_R):
            self.raw_tof_anchor = None
            return False
        if self.raw_tof_anchor is None:
            self.raw_tof_anchor = dist
            self.raw_tof_anchor_t = t
            return False
        if abs(dist - self.raw_tof_anchor) > self.TOF_STUCK_EPS:
            self.raw_tof_anchor = dist
            self.raw_tof_anchor_t = t
            return False
        return (t - self.raw_tof_anchor_t) >= self.GO_RAW_STUCK_T

    def _side_stuck(self, t, sl, sr):
        """Side sonar pinned very close and flat while driving forward.

        A box beside the body is not enough on its own (driving parallel to a
        wall keeps a constant sonar range).  Only a *very* close, unchanging
        reading counts, which means the chassis is actually scraping.
        """
        m = min(sl, sr)
        if not math.isfinite(m) or m > self.SIDE_STUCK_R:
            self.side_anchor = None
            return False
        if self.side_anchor is None:
            self.side_anchor = m
            self.side_anchor_t = t
            return False
        if abs(m - self.side_anchor) > self.SIDE_STUCK_EPS:
            self.side_anchor = m
            self.side_anchor_t = t
            return False
        return (t - self.side_anchor_t) >= self.SIDE_STUCK_T

    def _turn_stuck(self, t):
        """IMU yaw flat while a turn is commanded (wheels not turning)."""
        if self.rel_anchor is None:
            self.rel_anchor = self.rel
            self.rel_anchor_t = t
            return False
        if abs(wrap(self.rel - self.rel_anchor)) > self.TURN_STUCK_EPS:
            self.rel_anchor = self.rel
            self.rel_anchor_t = t
            return False
        return (t - self.rel_anchor_t) >= self.TURN_STUCK_T

    def _reset_stuck(self):
        self.tof_anchor = None
        self.raw_tof_anchor = None
        self.side_anchor = None
        self.rel_anchor = None

    def _check_stuck(self, t, sl, sr):
        """Sensor-only progress check run once per control tick.

        Called for every state except FINAL / REVERSE / ESCAPE / STRAIGHTEN.
        Uses the command issued on the previous tick, so it works in any mode
        and does not depend on wheel encoders.
        """
        v, w = self.last_v_cmd, self.last_w_cmd
        fwd = v > 0.02
        turning = abs(w) > 0.35 and not fwd
        reason = None
        if turning:
            if self._turn_stuck(t):
                reason = 'IMU yaw flat %.1f s' % self.TURN_STUCK_T
            else:
                self.tof_anchor = None
                self.raw_tof_anchor = None
                self.side_anchor = None
        elif fwd:
            # Watch only the body-width corridor: a box being passed beside the
            # chassis keeps a nearly constant side range and must not look like
            # a forward stall.  `_path_obstacle` also drops wall returns.
            po = self._path_obstacle(t, half_width=0.130, max_r=1.20)
            dist = po[2] if po is not None else INF
            if self._raw_tof_stuck(t):
                reason = ('GO raw TOF flat %.1f s @ %.2f m'
                          % (self.GO_RAW_STUCK_T, self.raw_d))
            elif abs(w) < 0.45 and self._tof_stuck(t, dist, True):
                reason = ('front TOF flat %.1f s @ %.2f m'
                          % (self.TOF_STUCK_T, dist))
            elif self._side_stuck(t, sl, sr):
                reason = ('side sonar pressed flat %.1f s @ %.2f m'
                          % (self.SIDE_STUCK_T, min(sl, sr)))
            else:
                self.rel_anchor = None
        else:
            self._reset_stuck()
        if reason is None:
            return False
        print('[nav] stuck in %s (%s) -> escape' % (self.state, reason),
              flush=True)
        self._enter('ESCAPE', t)
        return True

    # --------------------------------------------------------------- turning
    def _turn_to(self, target, t):
        err = wrap(target - self.rel)
        if abs(err) < self.ALIGN_TOL:
            if self.ok_since is None:
                self.ok_since = t
            return 0.0, (t - self.ok_since) >= self.SETTLE
        self.ok_since = None
        w = clamp(self.K_TURN * err, -self.TURN_W_MAX, self.TURN_W_MAX)
        if abs(w) < self.TURN_W_MIN:
            w = math.copysign(self.TURN_W_MIN, w)
        return w, False

    def _drive_heading(self, target, v_max=None):
        err = wrap(target - self.rel)
        v = self.V_FWD if v_max is None else v_max
        v *= 1.0 - min(0.85, abs(err) / 0.6)
        w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)
        return v, w

    def _side_guard(self, w, sl, sr):
        """Gentle push away from a close side obstacle.  It is deliberately
        weak and only active inside SIDE_GUARD, so it can never override the
        heading hold the way a 0.55 m balancing rule did."""
        g = self.SIDE_GUARD
        if sl < g:
            w -= 0.8 * (g - sl) / g
        if sr < g:
            w += 0.8 * (g - sr) / g
        return clamp(w, -self.W_MAX, self.W_MAX)

    @staticmethod
    def _wall_guard(w, lat):
        margin = ColorCourseNavigator.WALL_SIDE - 0.130
        if lat > margin:
            w -= clamp(2.0 * (lat - margin), 0.0, 0.50)
        elif lat < -margin:
            w += clamp(2.0 * (-lat - margin), 0.0, 0.50)
        return clamp(w, -ColorCourseNavigator.W_MAX,
                     ColorCourseNavigator.W_MAX)

    @staticmethod
    def _wall_hard(w, lat):
        margin = ColorCourseNavigator.WALL_SIDE - 0.140
        if lat > margin:
            w = min(w, -0.60)
        elif lat < -margin:
            w = max(w, +0.60)
        return clamp(w, -ColorCourseNavigator.W_MAX,
                     ColorCourseNavigator.W_MAX)

    # ------------------------------------------------------- scan target
    def _choose_target(self, rel):
        """Most open corridor, biased toward the finish direction.

        Returns (target_abs, clear, box_limited):
          clear        free distance along that bearing (TOF, walls included)
          box_limited  True when a BOX (not a wall / open space) ends the run
        """
        if not self.scan:
            return 0.0, self.MAX_RANGE, False
        win = max(1, int(round(self.SCAN_WINDOW / self.BIN)))
        best = None
        for b in self.scan:
            ang = b * self.BIN
            if abs(ang) > self.SWEEP_AMP + 0.01:
                continue
            heading = wrap(rel + ang)
            if abs(heading) > self.SCAN_FWD_LIMIT:
                continue
            vals = []
            wall_d = INF
            for k in range(-win, win + 1):
                rr = self.scan.get(b + k)
                if rr is None:
                    continue
                aa = (b + k) * self.BIN
                if self._is_side_wall(rr, aa) or self._is_wall(rr, aa):
                    wall_d = min(wall_d, rr)
                    continue
                vals.append(rr)
            obs_d = min(vals) if vals else self.MAX_RANGE
            clear = min(obs_d, wall_d, self.MAX_RANGE)
            box_lim = obs_d <= wall_d and obs_d < self.MAX_RANGE - 0.05
            score = clear - self.SCAN_BIAS * abs(heading)
            if best is None or score > best[0]:
                best = (score, heading, clear, box_lim)
        if best is None:
            return 0.0, self.MAX_RANGE, False
        return best[1], best[2], best[3]

    def _lateral_clear(self, side, need):
        """True when sliding `need` sideways stays clear of boxes and walls.

        Uses the scan hit points (box surfaces) from the current sweep, so a
        box that is off to the side but invisible to the straight beam still
        blocks the wrong detour side.
        """
        p0 = self.progress(self.x, self.y)
        l0 = self.lateral(self.x, self.y)
        margin = 0.120
        for frac in (0.25, 0.50, 0.75, 1.0):
            l = l0 + side * need * frac
            if abs(l) > self.WALL_SIDE - margin:
                return False
            for qp, ql in self.mem_pts:
                if abs(qp - p0) > 0.30:
                    continue
                if (qp - p0) ** 2 + (ql - l) ** 2 < margin ** 2:
                    return False
        return True

    def _free_side(self, lat):
        """Pick LEFT (+1) or RIGHT (-1) from wall room and scan openness."""
        room_l = self.WALL_SIDE - lat
        room_r = self.WALL_SIDE + lat
        if room_l > room_r + 0.10:
            return +1
        if room_r > room_l + 0.10:
            return -1

        def openness(ang):
            b = int(round(ang / self.BIN))
            vals = [self.scan[b + k] for k in range(-3, 4) if (b + k) in self.scan]
            return min(vals) if vals else 0.0

        a = math.radians(50)
        return +1 if openness(+a) >= openness(-a) else -1

    # ------------------------------------------------------------- main entry
    def step(self, x, y, rel, t, sonar_left=INF, sonar_right=INF,
             finish=False):
        self.x, self.y, self.rel = x, y, rel
        if self.t_state == 0.0:
            self.t_state = t
        fr0 = self._front(t)
        self.front_d = fr0[2] if fr0 is not None else INF
        rf = self._raw_front(t, 0.30)
        self.raw_d = rf[2] if rf is not None else INF
        if self.raw_d >= self.MAX_RANGE - 0.05:
            self.raw_d = INF            # saturated / no return: not a signal
        if finish:
            self.finish_seen = True
            self.last_finish_t = t

        if self.state is None:
            print('[nav] mode = %s' % self.mode, flush=True)
            self._enter_cycle(t)
        if self.finished:
            return 0.0, 0.0

        s = self.state
        elapsed = t - self.t_state
        prog = self.progress(x, y)
        lat = self.lateral(x, y)

        # Open round: count travel along the ring.  x/y are dead reckoned from
        # the commanded forward speed, so in-place corner turns add nothing.
        if self.mode == 'open':
            ds = math.hypot(x - self.open_prev_x, y - self.open_prev_y)
            self.open_prev_x, self.open_prev_y = x, y
            if s == 'O_FOLLOW':
                self.open_dist += ds
                if self.open_start_t is None and self.open_corners == 0:
                    # First O_FOLLOW step after the mandatory hold: this is
                    # the instant the robot actually leaves the start point.
                    self.open_start_t = t

        if self.DEBUG and int(t * 2) != self._dbg:
            self._dbg = int(t * 2)
            print('[nav] %-9s %-10s p=%.2f lat=%+.2f rel=%+5.1fdeg fin=%d '
                  'sonar L=%.2f R=%.2f tgt=%+5.1fdeg'
                  % (self.mode, s, prog, lat, math.degrees(rel), finish,
                     sonar_left, sonar_right, math.degrees(self.target_abs)),
                  flush=True)

        if self.mode != 'open' and s != 'FINAL':
            if self.finish_seen or prog >= self.line_min_p:
                print('[nav] finish zone -> FINAL', flush=True)
                self._enter('FINAL', t)
                s = 'FINAL'
                elapsed = 0.0

        # Sensor-only stall check: catches a box/wall the state handlers did
        # not anticipate.  REVERSE / ESCAPE / STRAIGHTEN already move the
        # robot out of trouble, so they are excluded.  The open round uses its
        # own wall geometry check below.
        if self.mode != 'open' and s not in ('FINAL', 'REVERSE', 'ESCAPE',
                                             'STRAIGHTEN'):
            if self._check_stuck(t, sonar_left, sonar_right):
                s = self.state
                elapsed = 0.0

        handler = getattr(self, '_s_' + s.lower(), None)
        if handler is None:
            return 0.0, 0.0
        v, w = handler(t, elapsed, prog, lat, sonar_left, sonar_right)
        self.last_v_cmd, self.last_w_cmd = v, w
        return v, w

    # ---------------------------------------------------------------- helpers
    def _enter(self, state, t):
        self.state = state
        self.t_state = t
        self.ok_since = None
        self.sx, self.sy = self.x, self.y
        self.lat_entry = self.lateral(self.x, self.y)
        self.tof_anchor = None
        self.raw_tof_anchor = None
        self.side_anchor = None
        self.rel_anchor = None
        if state == 'SCAN':
            self.scan_t0 = t
            self.scan_active = True
            self.scan = {}
            self.mem_pts = []
        elif state == 'REVERSE':
            self.reverse_count += 1

    def _enter_cycle(self, t):
        """Start (or restart) the main loop of the selected algorithm."""
        if self.mode == 'open':
            self._enter('O_FOLLOW', t)
        elif self.mode == 'bug2':
            self._enter('GO', t)
        elif self.mode == 'astar' and self.obstacles:
            self._enter('PLAN', t)
        else:
            self._enter('SCAN', t)

    # ======================================================= A* mode states
    def _s_plan(self, t, el, prog, lat, sl, sr):
        path = self._plan_path()
        if path and len(path) >= 2:
            self.path = path
            self.path_i = 0
            print('[nav] A*: %d waypoints toward the finish' % len(path),
                  flush=True)
            self._enter('FOLLOW', t)
        else:
            print('[nav] A*: no path -> reactive scan-and-go this cycle',
                  flush=True)
            self._enter('SCAN', t)
        return 0.0, 0.0

    def _s_follow(self, t, el, prog, lat, sl, sr):
        if not self.path:
            self._enter('PLAN', t)
            return 0.0, 0.0
        tgt, seg = self._path_target(self.x, self.y)
        self.path_i = seg
        d_end = math.hypot(self.path[-1][0] - self.x, self.path[-1][1] - self.y)
        if d_end <= self.WP_TOL:
            print('[nav] plan complete -> final approach', flush=True)
            self._enter('FINAL', t)
            return 0.0, 0.0

        heading = math.atan2(tgt[1] - self.y, tgt[0] - self.x)
        err = wrap(heading - self.rel)
        v = self.V_FWD * (1.0 - min(0.75, abs(err) / 1.10))
        w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)

        po = self._path_obstacle(t, half_width=0.060, max_r=0.20)
        if po is not None and po[2] <= 0.035:
            print('[nav] plan blocked -> decide pass side', flush=True)
            self.decide_ref = po[0]
            self._enter('DECIDE', t)
            return 0.0, 0.0

        w = self._wall_guard(w, lat)
        w = self._wall_hard(w, lat)
        side_min = min(sl, sr)
        if side_min < self.SIDE_STOP:
            v = 0.0
        elif side_min < 0.060:
            v *= 0.65
        return v, w

    # ================================================= scan-and-go states
    def _s_scan(self, t, el, prog, lat, sl, sr):
        if el < self.SCAN_TOTAL + 0.10:
            return 0.0, 0.0
        self._remember_scan(self.rel)
        tgt, clear, box_lim = self._choose_target(self.rel)
        self.target_abs = tgt
        self.scan_target_rel = wrap(tgt - self.rel)

        # Free run length along the chosen bearing: up to DECIDE_DIST short of
        # whatever ends the corridor, never leaving the lane.
        if clear < self.MAX_RANGE - 0.05:
            run = clear - self.DECIDE_DIST
        else:
            run = self.MAX_RUN
        run = min(run, self.MAX_RUN)
        sa = math.sin(tgt)
        if sa > 1e-3:
            run = min(run, (self.LANE_LIM - lat) / sa)
        elif sa < -1e-3:
            run = min(run, (-self.LANE_LIM - lat) / sa)
        self.run_len = max(0.0, run)
        self.run_box = bool(box_lim)

        print('[nav] scan: %d bearings -> target %+.0f deg (%s), clear %.2f m, '
              'run %.2f m%s'
              % (len(self.scan), math.degrees(self.scan_target_rel),
                 'LEFT' if self.scan_target_rel > self.STRAIGHT_TOL else
                 'RIGHT' if self.scan_target_rel < -self.STRAIGHT_TOL
                 else 'STRAIGHT', clear, self.run_len,
                 ' (ends at a box)' if self.run_box else ''), flush=True)

        if self.run_len < self.MIN_RUN:
            if abs(self.scan_target_rel) <= self.STRAIGHT_TOL and box_lim:
                # Box dead ahead with no room: straight-box rule.
                self._enter('DECIDE', t)
                return 0.0, 0.0
            print('[nav] no usable direction -> reverse', flush=True)
            self._enter('REVERSE', t)
            return 0.0, 0.0
        self._enter('TURN_GO', t)
        return 0.0, 0.0

    def _s_turn_go(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(self.target_abs, t)
        if done or el > self.T_TURN:
            self._enter('APPROACH', t)
            return 0.0, 0.0
        return 0.0, w

    def _s_approach(self, t, el, prog, lat, sl, sr):
        fr = self._front(t, 0.60)
        dc = fr[2] if fr is not None else INF
        po = self._path_obstacle(t, half_width=0.090, max_r=0.60)
        side_min = min(sl, sr)
        moved = math.hypot(self.x - self.sx, self.y - self.sy)

        # A box scraping the body (not the wall): treat it as the box to pass.
        sb = self._side_box(sl, sr, lat)
        if side_min < self.SIDE_REVERSE and sb != 0:
            self.decide_ref = sb * math.pi / 2.0
            print('[nav] box scrape (L=%.2f R=%.2f) -> decide pass side'
                  % (sl, sr), flush=True)
            self._enter('DECIDE', t)
            return 0.0, 0.0

        if dc <= self.DECIDE_DIST \
                or (po is not None and po[2] <= self.DECIDE_DIST):
            self._enter('DECIDE', t)
            return 0.0, 0.0
        if moved >= self.run_len - 0.01:
            if self.run_box:
                self._enter('DECIDE', t)
            else:
                print('[nav] run finished in open space -> scan again',
                      flush=True)
                self._enter_cycle(t) if self.mode == 'astar' \
                    else self._enter('SCAN', t)
            return 0.0, 0.0
        if el > self.T_APPROACH:
            print('[nav] approach timeout -> re-scan', flush=True)
            self._enter('SCAN', t)
            return 0.0, 0.0

        face = wrap(0.0 - self.rel)
        if abs(face) > math.radians(85):
            return 0.0, clamp(2.2 * face, -self.TURN_W_MAX, self.TURN_W_MAX)

        # Hold the scan bearing (servo stays straight).
        err = wrap(self.target_abs - self.rel)
        w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)
        v = self.V_FWD * (1.0 - min(0.85, abs(err) / 0.6))
        # Slow down near a box with ONE factor (never stack two slow-downs:
        # the robot would crawl below the stuck-detector speed threshold).
        slow = 1.0
        if po is not None and po[2] < 0.45:
            slow = min(slow, po[2] / 0.60)
        if dc < 0.40:
            slow = min(slow, (dc - 0.06) / 0.34)
        v = max(v * max(0.40, slow), 0.040)

        if side_min < self.SIDE_HARD:
            w = -0.7 * self.W_MAX if sl < sr else 0.7 * self.W_MAX
        w = self._side_guard(w, sl, sr)
        w = self._wall_guard(w, lat)
        w = self._wall_hard(w, lat)

        if abs(lat) > self.WALL_SIDE - 0.140:
            v *= 0.50
        if side_min < self.SIDE_STOP:
            v = 0.0
        elif side_min < 0.120:
            v *= 0.65

        return v, w

    def _s_escape(self, t, el, prog, lat, sl, sr):
        """Reverse away from a stall while probing for free space.

        A side sonar pinched below SIDE_STOP aborts immediately (reversing
        further would drag the body along the obstacle).  Otherwise back up
        until the front TOF and both side sonars show room, with a nominal
        1.4 s and a 2.4 s hard limit.
        """
        if min(sl, sr) < self.SIDE_STOP:
            print('[nav] escape abort: side pinched -> straighten and re-scan',
                  flush=True)
            self._enter('STRAIGHTEN', t)
            return 0.0, 0.0
        fr = self._front(t, 0.35)
        front = fr[2] if fr is not None else INF
        front_clear = front > self.ESCAPE_FRONT_CLEAR
        side_clear = min(sl, sr) > self.ESCAPE_SIDE_CLEAR
        if ((el >= self.ESCAPE_MIN_T and front_clear and side_clear)
                or (el >= self.ESCAPE_T and front_clear)
                or el >= self.ESCAPE_MAX_T):
            print('[nav] escape done (front=%.2f L=%.2f R=%.2f) -> re-scan'
                  % (front, sl, sr), flush=True)
            self._enter('STRAIGHTEN', t)
            return 0.0, 0.0
        return -self.V_REVERSE, 0.0

    def _s_reverse(self, t, el, prog, lat, sl, sr):
        dur = self.T_REVERSE * (1.0 + 0.5 * min(self.reverse_count - 1, 3))
        if el < dur:
            return -self.V_REVERSE, 0.0
        print('[nav] reverse done -> straighten and scan again', flush=True)
        self._enter('STRAIGHTEN', t)
        return 0.0, 0.0

    def _s_straighten(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(0.0, t)
        if done or el > self.T_STRAIGHTEN:
            self._enter_cycle(t)
            return 0.0, 0.0
        return 0.0, w

    def _s_decide(self, t, el, prog, lat, sl, sr):
        """Box is right ahead.  Pass side comes from the scan bearing:
             target LEFT  -> turn RIGHT, detour, turn LEFT (straighten)
             target RIGHT -> turn LEFT,  detour, turn RIGHT
             target STRAIGHT -> scan again; after STRAIGHT_RESCANS pass on the
                                side the box is NOT on / the more open side.
        A box clearly off-axis (> OVERRIDE_BEARING) is always passed away from
        that side so the detour can never turn into it."""
        fr = self._front(t, 0.60)
        po = self._path_obstacle(t, half_width=0.090, max_r=0.60)
        src = po if po is not None else fr

        ref = self.decide_ref
        if ref is None:
            ref = self.scan_target_rel
            if src is not None and abs(src[0]) > self.OVERRIDE_BEARING:
                ref = src[0]
        self.decide_ref = None

        if abs(ref) <= self.STRAIGHT_TOL:
            if self.straight_scans < self.STRAIGHT_RESCANS:
                self.straight_scans += 1
                print('[nav] box straight ahead -> stop and scan again',
                      flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0
            b = src[0] if src is not None else 0.0
            if b > 0.03:
                side = -1
            elif b < -0.03:
                side = +1
            else:
                side = self._free_side(lat)
            print('[nav] box still straight -> pass %s (rescan limit)'
                  % ('RIGHT' if side < 0 else 'LEFT'), flush=True)
        else:
            side = -1 if ref > 0.0 else +1
        self.straight_scans = 0

        # The last detour on this side was blocked by another box / too small
        # a gap: try the other side instead of repeating the same failure.
        if self.blocked_side == side:
            print('[nav] %s detour was blocked last time -> try the other side'
                  % ('LEFT' if side > 0 else 'RIGHT'), flush=True)
            side = -side
        self.blocked_side = 0

        room_l = self.WALL_SIDE - lat
        room_r = self.WALL_SIDE + lat
        if side > 0 and room_l < 0.180 and room_r > room_l:
            print('[nav] left detour blocked by wall -> mirror right', flush=True)
            side = -1
        elif side < 0 and room_r < 0.180 and room_l > room_r:
            print('[nav] right detour blocked by wall -> mirror left', flush=True)
            side = +1

        # The scan only reports the box ahead, so a second box may already be
        # sitting on the intended detour side.  The side sonars are the veto:
        # turn toward the freer side instead of driving into it.
        if side < 0 and sr + 0.040 < sl and sr < 0.260:
            print('[nav] right side blocked (L=%.2f R=%.2f) -> pass LEFT'
                  % (sl, sr), flush=True)
            side = +1
        elif side > 0 and sl + 0.040 < sr and sl < 0.260:
            print('[nav] left side blocked (L=%.2f R=%.2f) -> pass RIGHT'
                  % (sl, sr), flush=True)
            side = -1

        # The box veto must not push the detour into a wall.
        room_l = self.WALL_SIDE - lat
        room_r = self.WALL_SIDE + lat
        if side > 0 and room_l < 0.180 and room_r > room_l:
            print('[nav] left detour still blocked by wall -> pass RIGHT',
                  flush=True)
            side = -1
        elif side < 0 and room_r < 0.180 and room_l > room_r:
            print('[nav] right detour still blocked by wall -> pass LEFT',
                  flush=True)
            side = +1

        box_rel = src[1] * math.sin(self.rel + src[0]) if src is not None else 0.0
        self.box_rel = clamp(box_rel, -0.22, 0.22)
        self.side_need = clamp(side * self.box_rel + self.SIDE_CLEAR_BOX,
                               self.PASS_CLEAR_MIN, self.PASS_LAT_MAX)

        # Scan memory can veto a detour that the front beam cannot see: a
        # second box may sit just off to the side of the chosen path.
        if not self._lateral_clear(side, self.side_need):
            other = -side
            need_other = clamp(other * self.box_rel + self.SIDE_CLEAR_BOX,
                               self.PASS_CLEAR_MIN, self.PASS_LAT_MAX)
            if self._lateral_clear(other, need_other):
                print('[nav] %s detour blocked by a scanned box -> pass %s'
                      % ('LEFT' if side > 0 else 'RIGHT',
                         'RIGHT' if other < 0 else 'LEFT'), flush=True)
                side = other
                self.side_need = need_other
            else:
                print('[nav] both lateral paths blocked -> reverse', flush=True)
                self.blocked_side = side
                self._enter('REVERSE', t)
                return 0.0, 0.0

        self.maneuvers += 1
        self.pass_side = side
        ph = wrap(self.rel + side * self.DETOUR_ANGLE)
        if abs(ph) > math.radians(95):
            ph = math.copysign(math.radians(95), ph)
        self.pass_heading = ph
        print('[nav] pass #%d: ref %+.0f deg -> turn %s (toward middle), '
              'detour %.2f m (need %.2f)'
              % (self.maneuvers, math.degrees(ref),
                 'RIGHT' if side < 0 else 'LEFT',
                 self.DETOUR_DIST, self.side_need), flush=True)
        self._enter('TURN_SIDE', t)
        return 0.0, 0.0

    def _s_turn_side(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(self.pass_heading, t)
        if done or el > self.T_TURN:
            self._enter('SIDE_MOVE', t)
            return 0.0, 0.0
        return 0.0, w

    def _s_side_move(self, t, el, prog, lat, sl, sr):
        moved = math.hypot(self.x - self.sx, self.y - self.sy)
        dlat = (lat - self.lat_entry) * self.pass_side
        room = self.WALL_SIDE - lat * self.pass_side
        fr = self._front(t, 0.45)
        po = self._path_obstacle(t, half_width=0.090, max_r=0.45)
        ahead_d = min(fr[2] if fr is not None else INF,
                      po[2] if po is not None else INF)
        pass_sonar = sl if self.pass_side > 0 else sr
        # Stop once the lateral offset is reached OR the diagonal has run its
        # bounded distance; the side sonar guard below protects the detour.
        if (dlat >= self.side_need
                or moved >= self.DETOUR_DIST
                or dlat >= self.PASS_LAT_MAX
                or room <= 0.100 or el > self.T_SIDE):
            print('[nav] detour done: moved=%.2f dlat=%+.2f need=%.2f'
                  % (moved, dlat, self.side_need), flush=True)
            self._enter('TURN_BACK', t)
            return 0.0, 0.0
        # Something is already pressing on the detour side: stop pushing into
        # it.  Back out if the detour has barely begun, otherwise straighten.
        if pass_sonar < 0.045:
            if moved < self.DETOUR_MIN_MOVE:
                print('[nav] detour side blocked (sonar=%.2f) -> reverse'
                      % pass_sonar, flush=True)
                self.blocked_side = self.pass_side
                self._enter('REVERSE', t)
            else:
                print('[nav] detour side tight (sonar=%.2f) -> straighten'
                      % pass_sonar, flush=True)
                self._enter('TURN_BACK', t)
            return 0.0, 0.0
        if ahead_d <= 0.090 and moved < self.DETOUR_MIN_MOVE:
            print('[nav] detour blocked (ahead=%.2f) -> reverse' % ahead_d,
                  flush=True)
            self.blocked_side = self.pass_side
            self._enter('REVERSE', t)
            return 0.0, 0.0
        v, w = self._drive_heading(self.pass_heading, self.V_PASS)
        if ahead_d < 0.30:
            v *= max(0.35, (ahead_d - 0.06) / 0.24)
        side_min = min(sl, sr)
        if side_min < self.SIDE_STOP:
            v = 0.0
        elif side_min < 0.070:
            v *= 0.60
        return v, self._wall_hard(clamp(w, -self.W_MAX, self.W_MAX), lat)

    def _s_turn_back(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(0.0, t)
        if done or el > self.T_TURN:
            self._enter('CROSS', t)
            return 0.0, 0.0
        return 0.0, w

    def _s_cross(self, t, el, prog, lat, sl, sr):
        moved = math.hypot(self.x - self.sx, self.y - self.sy)
        if moved >= self.CROSS_MIN or el > self.CROSS_MAX_T:
            print('[nav] crossed (moved=%.2f) -> next cycle' % moved, flush=True)
            self.reverse_count = 0
            self.blocked_side = 0
            self._enter_cycle(t)
            return 0.0, 0.0
        fr = self._front(t, 0.45)
        po = self._path_obstacle(t, half_width=0.090, max_r=0.35)
        if (fr is not None and fr[2] <= self.DECIDE_DIST) \
                or (po is not None and po[2] <= self.DECIDE_DIST):
            print('[nav] new box directly ahead -> scan again', flush=True)
            self._enter('SCAN', t)
            return 0.0, 0.0
        v, w = self._drive_heading(0.0, self.V_PASS)
        w = self._side_guard(w, sl, sr)
        w = self._wall_guard(w, lat)
        w = self._wall_hard(w, lat)
        return v, w

    # ============================================================ Bug2 states
    def _s_go(self, t, el, prog, lat, sl, sr):
        po = self._path_obstacle(t, half_width=0.110, max_r=0.60, cone=0.95)
        fc = po[2] if po is not None else INF
        sb = self._side_box(sl, sr, lat, thresh=self.BUG_HIT_SIDE)
        if fc < self.BUG_HIT_FRONT or sb != 0:
            self._bug_hit(fc, prog, lat, sl, sr, t)
            return 0.0, 0.0
        target = clamp(-self.K_LAT * lat, -self.LAT_HEAD_MAX, self.LAT_HEAD_MAX)
        err = wrap(target - self.rel)
        v = self.V_FWD * (1.0 - min(1.0, abs(err) / 0.5))
        w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)
        w = self._wall_hard(self._wall_guard(w, lat), lat)
        if fc < 0.40:
            v *= max(0.30, (fc - 0.06) / 0.34)
        return v, w

    def _bug_hit(self, fc, prog, lat, sl, sr, t):
        self.hit_p, self.hit_rel, self.hit_lat = prog, self.rel, lat
        self.off_line = False
        self.hits += 1
        sb = self._side_box(sl, sr, lat, thresh=0.20)
        if sb != 0:
            self.obs_side = sb                  # box is on that side
        else:
            self.obs_side = -1 if lat <= 0.0 else +1   # go where there is room
        self.lost_xy = None
        print('[nav] hit #%d (p=%.2f lat=%.2f fc=%.2f) -> box on the %s, '
              'pass on the %s'
              % (self.hits, prog, lat, fc,
                 'left' if self.obs_side > 0 else 'right',
                 'right' if self.obs_side > 0 else 'left'), flush=True)
        self._enter('B_ALIGN', t)

    def _bug_side(self, sl, sr):
        return sl if self.obs_side > 0 else sr

    def _s_b_align(self, t, el, prog, lat, sl, sr):
        ds = self._bug_side(sl, sr)
        fr = self._front(t, 0.30)
        fc = fr[2] if fr is not None else INF
        turned = abs(wrap(self.rel - self.hit_rel))
        if ((fc > self.CLEAR_F and ds < self.FOLLOW_SEE
             and turned > self.ALIGN_MIN)
                or turned > self.ALIGN_MAX or el > self.T_TURN):
            self._enter('B_FOLLOW', t)
            self.lost_xy = None
            return 0.0, 0.0
        return 0.0, -self.obs_side * self.ALIGN_W

    def _s_b_follow(self, t, el, prog, lat, sl, sr):
        if el > self.T_FOLLOW or -self.obs_side * lat > self.BUG_LANE_HALF:
            self.uturn_target = wrap(self.rel + math.pi)
            print('[nav] lane edge / timeout while following -> reverse '
                  'direction', flush=True)
            self._enter('B_UTURN', t)
            return 0.0, 0.0
        if abs(lat - self.hit_lat) > self.OFF_LINE:
            self.off_line = True
        ds = self._bug_side(sl, sr)
        # Wide cone: a box at the body corner while following must count as
        # blocked, otherwise the body drives through it unseen.
        fr = self._front(t, 0.75)
        fc = fr[2] if fr is not None else INF
        front_blocked = fc < self.INNER_FRONT
        lost = ds > self.LOST and not front_blocked
        gone = 0.0
        if lost:
            if self.lost_xy is None:
                self.lost_xy = (self.x, self.y)
            gone = math.hypot(self.x - self.lost_xy[0], self.y - self.lost_xy[1])
        else:
            self.lost_xy = None
        back_on_line = (self.off_line and abs(lat) < self.M_TOL
                        and prog > self.hit_p + self.GAIN
                        and fc > self.CLEAR_F)
        if (lost and gone >= self.LOST_LEAVE) or back_on_line:
            print('[nav] leave obstacle (p=%.2f lat=%.2f)' % (prog, lat),
                  flush=True)
            self._enter('B_LEAVE', t)
            return 0.0, 0.0
        if front_blocked:
            return 0.0, -self.obs_side * self.CORNER_TURN_W
        if lost:
            if gone < self.PASS_STRAIGHT:
                v, w = self.V_FOLLOW, 0.0
            else:
                v, w = self.V_CORNER, self.obs_side * self.W_CORNER
        else:
            w = clamp(self.obs_side * self.K_WALL * (ds - self.D_FOLLOW),
                      -self.W_MAX, self.W_MAX)
            v = self.V_FOLLOW * (1.0 - 0.5 * min(1.0, abs(w) / self.W_MAX))
        return v, w

    def _s_b_uturn(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(self.uturn_target, t)
        if done or el > 2 * self.T_TURN:
            self.obs_side = -self.obs_side
            self.lost_xy = None
            self._enter('B_FOLLOW', t)
            return 0.0, 0.0
        return 0.0, w

    def _s_b_leave(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(0.0, t)
        if done or el > self.T_TURN:
            self.reverse_count = 0
            self._enter('GO', t)
            return 0.0, 0.0
        return 0.0, w

    # ===================================================== OPEN ROUND states
    def _open_turn_sign(self):
        """+1 = left turns (counter-clockwise), -1 = right turns (clockwise)."""
        return -1.0 if self.open_clockwise else 1.0

    def _s_o_follow(self, t, el, prog, lat, sl, sr):
        turn_sign = self._open_turn_sign()
        out_side = -turn_sign
        inner_side = turn_sign
        inner_d = sl if inner_side > 0 else sr
        outer_d = sr if out_side < 0 else sl

        # Remember the forward TOF at the start point.  In the open round the
        # servo is held straight, so this is the range to the end wall of the
        # starting side.  After three laps the robot is on the same side
        # heading the same way, and the same wall gives the same reading: an
        # absolute along-track position, independent of speed and clock drift.
        if (self.open_start_tof is None and self.open_corners == 0
                and math.isfinite(self.raw_d)):
            self.open_start_tof = self.raw_d
            print('[nav] open: start TOF %.3f m' % self.open_start_tof,
                  flush=True)

        # 3 complete laps, stopped at the remembered start point.  Primary
        # stop: the front TOF matches the start reading (same end wall).
        # Clock memory (A/B) remains as a fallback if the TOF reference was
        # never available, and as a last-resort guard.
        if self.open_final_leg_t is not None:
            stop = False
            why = ''
            if (self.open_start_tof is not None
                    and math.isfinite(self.raw_d)):
                if self.raw_d <= self.open_start_tof + self.OPEN_STOP_TOL:
                    self.open_tof_hits += 1
                else:
                    self.open_tof_hits = 0
                if self.open_tof_hits >= self.OPEN_STOP_HITS:
                    stop = True
                    why = ('TOF match %.3f m (start %.3f m)'
                           % (self.raw_d, self.open_start_tof))
            if not stop and self.open_final_leg_t0 is not None:
                el_final = t - self.open_final_leg_t0
                if self.open_start_tof is None:
                    # No TOF reference at all: use the clock exactly.
                    if el_final >= self.open_final_leg_t:
                        stop = True
                        why = ('clock fallback %.2f s'
                               % self.open_final_leg_t)
                elif el_final >= (self.open_final_leg_t
                                  + self.OPEN_STOP_EXTRA_T):
                    # TOF reference exists but never matched (sensor glitch):
                    # stop just past the clock estimate instead of running on.
                    stop = True
                    why = ('clock guard %.2f s'
                           % (self.open_final_leg_t + self.OPEN_STOP_EXTRA_T))
            if stop:
                self.finished = True
                self.finish_reason = (
                    'open round complete: %d laps at the start point (%s)'
                    % (self.OPEN_LAPS, why))
                print('[nav] OPEN ROUND COMPLETE: %d corners, %s, final leg '
                      '%.2f s (A=%.2f s, B=%.2f s)'
                      % (self.open_corners, why, self.open_final_leg_t,
                         self.open_first_leg_t or 0.0,
                         self.open_side_time or 0.0), flush=True)
                return 0.0, 0.0
        elif self.open_finish_from is not None:
            # Distance fallback if the clock memory was unavailable.
            if (self.open_dist - self.open_finish_from
                    >= self.open_finish_remaining):
                self.finished = True
                self.finish_reason = (
                    'open round complete: %d laps (%.1f m)'
                    % (self.OPEN_LAPS, self.open_dist))
                print('[nav] OPEN ROUND COMPLETE: %.2f m, %d corners'
                      % (self.open_dist, self.open_corners), flush=True)
                return 0.0, 0.0
        elif self.open_dist >= self.OPEN_TOTAL:
            # Fallback if a corner was never seen (start exactly on a corner).
            self.finished = True
            self.finish_reason = (
                'open round complete: %d laps (%.1f m)'
                % (self.OPEN_LAPS, self.open_dist))
            return 0.0, 0.0

        # The final leg never triggers another corner: after the 12th turn
        # the robot is on the starting side and only has to reach the start
        # point (TOF match / clock guard above).  This also covers a start
        # that is closer to the end wall than OPEN_CORNER_FRONT.
        if self.open_final_leg_t is None:
            # Reaching the outer wall ahead means the current side ends: turn
            # 90 deg toward the inner block and continue on the next side.
            front = self.raw_d
            if math.isfinite(front) and front < self.OPEN_CORNER_FRONT:
                # Clock memory (no encoders, no line).  A is the partial
                # first leg; each later interval is the pure drive time of
                # one full side (previous turn completed -> this corner
                # detected), so the turn duration itself is excluded from B.
                n = self.open_corners + 1
                if self.open_start_t is not None:
                    if self.open_last_turn_done_t is None:
                        self.open_first_leg_t = t - self.open_start_t
                    else:
                        self.open_corner_intervals.append(
                            t - self.open_last_turn_done_t)
                    if (n == 4 * self.OPEN_LAPS
                            and self.open_corner_intervals):
                        self.open_side_time = (
                            sum(self.open_corner_intervals)
                            / len(self.open_corner_intervals))
                self.open_heading = wrap(
                    self.open_heading + turn_sign * math.pi / 2.0)
                print('[nav] open: corner %d -> turn %s (front %.2f m)'
                      % (n, 'left' if turn_sign > 0 else 'right', front),
                      flush=True)
                self._enter('O_CORNER', t)
                return 0.0, 0.0

            # Stall guard: a wall much closer than the corner trigger while
            # the commanded speed produces no change means something is
            # blocking the chassis.  Back off briefly and resume.
            if math.isfinite(front) and front < self.OPEN_STALL_R:
                if self.open_stall_anchor is None:
                    self.open_stall_anchor = front
                    self.open_stall_t = t
                elif abs(front - self.open_stall_anchor) > self.OPEN_STALL_EPS:
                    self.open_stall_anchor = front
                    self.open_stall_t = t
                elif t - self.open_stall_t >= self.OPEN_STALL_T:
                    print('[nav] open: stall guard (front %.2f m) -> reverse'
                          % front, flush=True)
                    self.open_stall_anchor = None
                    self._enter('O_REVERSE', t)
                    return 0.0, 0.0
            else:
                self.open_stall_anchor = None

        # Heading hold along the current side, then centre in the 1 m track
        # with the outer-wall sonar; the inner block only needs a guard.
        v, w = self._drive_heading(self.open_heading, self.OPEN_V)
        if math.isfinite(outer_d) and outer_d < 0.90:
            w += out_side * self.OPEN_K_OUTER * (outer_d - self.OPEN_SIDE_TARGET)
        if math.isfinite(inner_d) and inner_d < 0.40:
            w += out_side * self.OPEN_K_INNER * (self.OPEN_INNER_SAFE - inner_d)

        return v, clamp(w, -self.W_MAX, self.W_MAX)

    def _s_o_corner(self, t, el, prog, lat, sl, sr):
        w, done = self._turn_to(self.open_heading, t)
        if done or el > self.OPEN_CORNER_TIMEOUT:
            self.open_corners += 1
            self.open_last_turn_done_t = t
            if self.open_corners == 1:
                self.open_first_leg = self.open_dist
            if self.open_corners % 4 == 0:
                print('[nav] open: lap %d/%d complete (%.2f m)'
                      % (self.open_corners // 4, self.OPEN_LAPS,
                         self.open_dist), flush=True)
            if self.open_corners == 4 * self.OPEN_LAPS:
                if (self.open_side_time is not None
                        and self.open_first_leg_t is not None):
                    # Prefer the measured clock: drive one full side minus
                    # the remembered partial first leg.
                    self.open_final_leg_t = max(
                        0.0, self.open_side_time - self.open_first_leg_t)
                    self.open_final_leg_t0 = t
                    print('[nav] open: 3 laps done, final leg %.2f s '
                          '(A=%.2f s, B=%.2f s)'
                          % (self.open_final_leg_t, self.open_first_leg_t,
                             self.open_side_time), flush=True)
                else:
                    # Distance fallback if the first corner was never timed.
                    first = (self.open_first_leg
                             if self.open_first_leg is not None
                             else self.OPEN_SIDE_LEN / 2.0)
                    self.open_finish_from = self.open_dist
                    self.open_finish_remaining = max(
                        0.0, self.OPEN_SIDE_LEN - first)
                    print('[nav] open: 3 laps done, %.2f m to the start line'
                          % self.open_finish_remaining, flush=True)
            self._enter('O_FOLLOW', t)
            return 0.0, 0.0
        return 0.0, w

    def _s_o_reverse(self, t, el, prog, lat, sl, sr):
        if el >= 0.70:
            self._enter('O_FOLLOW', t)
            return 0.0, 0.0
        return -self.V_REVERSE, 0.0

    # ============================================================ FINAL (all)
    def _s_final(self, t, el, prog, lat, sl, sr):
        # Creep in while easing back toward the lane centre: the reactive
        # modes can reach the finish zone well off to one side, and a centred
        # approach keeps the strip in view instead of sliding along the wall.
        target = clamp(-self.K_LAT * lat,
                       -self.LAT_HEAD_MAX, self.LAT_HEAD_MAX)
        herr = wrap(target - self.rel)
        if abs(herr) > math.radians(8):
            w, _ = self._turn_to(target, t)
            return 0.0, w

        if prog >= self.finish_p:
            self.finished = True
            self.finish_reason = ('finish line colour detected'
                                  if self.finish_seen else 'reached finish line')
            print('[nav] ON THE FINISH LINE (progress=%.2f m)' % prog, flush=True)
            return 0.0, 0.0

        raw = self._raw_front(t, 0.40)
        if raw is not None and raw[2] < self.WALL_STOP \
                and self._is_wall(raw[1], raw[0]):
            self.finished = True
            self.finish_reason = ('finish line crossed (safe stop before wall)'
                                  if self.finish_seen else 'end-wall safety stop')
            print('[nav] end-wall safety stop (range=%.2f m)' % raw[2],
                  flush=True)
            return 0.0, 0.0

        if el > self.T_FINAL:
            self.finished = True
            self.finish_reason = 'final-approach timeout'
            print('[nav] FINAL timeout -> safe stop', flush=True)
            return 0.0, 0.0

        v = self.V_FINAL
        if raw is not None and raw[2] < 0.35:
            v *= max(0.40, (raw[2] - 0.08) / 0.27)
        return v, clamp(self.K_HEAD * herr, -0.25, 0.25)
