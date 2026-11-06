"""Colour-course navigator for randomly scattered red/green boxes.

The arena contains eight boxes placed randomly on every launch, with at
least 0.30 m edge-to-edge clearance.  The robot does not have to touch every
box: it only has to reach the cyan finish line without hitting anything.

Control loop:

    SCAN       rotate the servo/TOF once, then pick the most open bearing
               (longest TOF return, biased toward the finish direction)
    APPROACH   drive along that bearing; steer with the TOF bearing and the
               camera colour centroid; stop just before a box
    CHECK      stand still, read the box colour with the angled camera
    red        turn LEFT 90 deg, sidestep until the right sonar has cleared
               the box, turn back RIGHT, cross, straighten with the IMU
    green      turn RIGHT 90 deg, sidestep until the left sonar has cleared
               the box, turn back LEFT, cross, straighten with the IMU
    then       SCAN again from the new position
    FINAL      cyan strip seen (or no coloured box in front near the wall):
               drive straight to the line and stop before the end wall

Angles are relative to the start heading: 0 = toward the finish, + = left.
Only simple sensor values are consumed, so the class is unit-testable.
"""

import heapq
import math

from autonomous_bot_sim.arena_layout import OBS_SIZE, PLAN_RADIUS, ARENA_HALF_W

INF = 9.9


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ColorCourseNavigator:
    # ---------------------------------------------------------------- arena
    # Start y = -1.35, finish strip y = +1.30, end-wall inner face y = +1.475.
    WALL_P        = 2.825       # end wall, measured from the start pose
    WALL_SIDE     = 0.475       # side wall, measured from the lane centre
    WALL_STOP     = 0.075       # raw TOF distance at which the wall guard stops
    BEAM_X        = 0.100       # TOF pivot ahead of the axle
    FINISH_P      = 2.600       # axle progress on the cyan strip
    LINE_MIN_P    = 2.250       # closest sensible start of the finish zone

    # ---------------------------------------------------------------- scan
    SWEEP_AMP     = 1.05        # servo sweep each side, rad
    BIN           = 0.05        # polar map resolution, rad
    SCAN_SETTLE   = 0.30
    SCAN_SWEEP    = 1.90
    SCAN_RETURN   = 0.30
    SCAN_TOTAL    = SCAN_SETTLE + SCAN_SWEEP + SCAN_RETURN
    MAX_RANGE     = 3.0
    SCAN_BIAS     = 0.55        # metre penalty per rad away from the finish
    SCAN_WINDOW   = 0.30        # rad: corridor width used to reject narrow gaps
    SCAN_FWD_LIMIT = math.radians(70)   # never aim outside this finish sector

    # Gentle sweep while driving: gives the TOF a bearing to the near box.
    APPROACH_AMP  = 1.05
    APPROACH_PER  = 1.40

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
    K_CAM         = 2.0
    W_MAX         = 0.85

    # ----------------------------------------------------- obstacle / colour
    CHECK_DIST    = 0.060       # forward TOF distance at which to stop
    COLOR_PIX     = 30          # colour pixels that count as "box seen"
    COLOR_WAIT    = 0.55

    # ---------------------------------------------------------- side sonars
    SIDE_GUARD    = 0.200
    SIDE_STOP     = 0.015
    SIDE_HARD     = 0.100       # override the target heading this close
    SIDE_REVERSE  = 0.035       # escape when almost touching
    SIDE_WALL_STOP = 0.120       # stop a sidestep this far from the side wall
    SIDE_CLEAR    = 0.400       # sonar reading that means the box is behind
    PASS_CLEAR_MIN = 0.140      # absolute minimum sidestep
    SIDE_CLEAR_BOX = 0.145      # required centre-to-box lateral clearance
    PASS_LAT_MAX  = 0.300       # never sidestep further than this

    # ------------------------------------------------------------- crossing
    CROSS_MIN     = 0.300
    CROSS_MAX_T   = 5.0

    # ---------------------------------------------------------- map planner
    # The obstacle layout is published by obstacle_randomizer.  A grid A* over
    # the known boxes produces a path that keeps PLAN_RADIUS clearance, so the
    # robot never has to squeeze through a gap narrower than its body.
    OBS_HALF      = OBS_SIZE / 2.0
    PLAN_DP       = 0.02
    PLAN_P_MIN    = -0.05
    PLAN_P_MAX    = 2.72
    PLAN_L_MAX    = ARENA_HALF_W - PLAN_RADIUS  # stay clear of the side walls
    PLAN_CLEAR    = PLAN_RADIUS  # guaranteed corridor radius (matches layout)
    PLAN_GOAL_P   = 2.45
    WP_TOL        = 0.070
    LOOKAHEAD     = 0.200       # smaller = tracks curves tighter, less corner cutting

    # ---------------------------------------------------------- recovery
    V_AVOID       = 0.060       # (unused) kept for tuning reference
    W_AVOID       = 0.90        # turn-in-place rate while escaping
    STUCK_T       = 2.0         # no progress for this long -> escape
    STUCK_DIST    = 0.040

    # -------------------------------------------------------------- timeouts
    T_APPROACH    = 8.0
    T_AVOID       = 0.9
    T_TURN        = 4.0
    T_SIDE        = 5.0
    T_CHECK       = 2.0
    T_FINAL       = 15.0

    # --------------------------------------------------------------- debug
    DEBUG         = True

    def __init__(self):
        # Known box centres in the navigator frame (progress, lateral).  Kept
        # across reset() because the layout is fixed for the whole run and may
        # arrive before START.  Empty -> fall back to pure scan navigation.
        self.obstacles = []
        self.reset(0.0, 0.0, 0.0)

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
        self.approach_heading = 0.0
        self.pass_heading = 0.0
        self.pass_color = None
        self.pass_side = 0
        self.avoid_side = -1.0
        self.side_need = self.PASS_CLEAR_MIN
        self.box_rel = 0.0
        self.side_flipped = False

        self.scan = {}
        self.scan_t0 = None
        self.scan_active = False
        self.sweep_t0 = None
        self.recent = []            # (t, bearing, range) for the front veto
        self.path = []
        self.path_i = 0

        self.check_red = 0
        self.check_green = 0
        self.side_seen = False
        self.last_color_t = -1e9
        self.sx = self.sy = 0.0
        self.lat_entry = 0.0
        self.ok_since = None
        self._dbg = -1

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
        """Give the navigator the box centres in its own (progress, lateral)
        frame.  Points are (progress, lateral) pairs."""
        self.obstacles = [(float(p), float(l)) for p, l in points]
        self.path = []
        self.path_i = 0
        if self.obstacles:
            print('[nav] map: %d obstacle centres received'
                  % len(self.obstacles), flush=True)

    def _clearance(self, p, l):
        """Edge distance from a point to the nearest box (metres)."""
        best = INF
        for op, ol in self.obstacles:
            dp = abs(p - op) - self.OBS_HALF
            dl = abs(l - ol) - self.OBS_HALF
            if dp < 0.0:
                dp = 0.0
            if dl < 0.0:
                dl = 0.0
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
            # Dead-reckoned pose may be a few cm off: snap to the nearest
            # legal cell (a valid path exists by construction).
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
                # Prefer the middle of a corridor: penalise low clearance.
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
        """Line-of-sight shortcutting: fewer, straighter waypoints."""
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
        """Pure-pursuit point: project the robot onto the path, then look
        LOOKAHEAD metres further along it.  Returns (point, segment_index).

        Projection (instead of chasing the next waypoint) means overshooting a
        corner cannot make the robot turn back toward a point behind it.
        """
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
                u = ((x - ax) * dx + (y - ay) * dy) / l2
                u = clamp(u, 0.0, 1.0)
            px, py = ax + u * dx, ay + u * dy
            d = math.hypot(x - px, y - py)
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
        """Full sweep while scanning, gentle sweep otherwise."""
        if self.scan_active:
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
            self.sweep_t0 = t
        if self.sweep_t0 is None:
            self.sweep_t0 = t
        return self.APPROACH_AMP * math.sin(
            2.0 * math.pi * (t - self.sweep_t0) / self.APPROACH_PER)

    def tof(self, bearing, rng, t):
        """Feed one TOF sample: servo joint angle (rad) and range (m).

        Bearings are stored in the fixed start frame so that cached samples
        stay valid while the body rotates during a detour.
        """
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

        Returns (bearing, range, forward_distance) or None.
        """
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
        """Nearest fresh TOF return including walls (for the end-wall guard)."""
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

    def _path_obstacle(self, t, half_width=0.18, max_r=1.00, cone=0.95):
        """Nearest box whose predicted lateral offset would hit the body.

        A box 0.11 m to the side of the centre line is still a collision for
        a 0.14 m body, even though the straight-ahead TOF ray misses it.
        Returns (bearing, range, forward_distance) or None.
        """
        best = None
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

    def _center(self, t):
        fr = self._front(t)
        return fr[2] if fr is not None else INF

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
        # If both sides see something, balance the gaps: this keeps the body
        # centred in a 30-40 cm corridor or a narrow lane next to the wall.
        near_l = sl < 0.55
        near_r = sr < 0.55
        if near_l and near_r:
            w += clamp(2.2 * (sl - sr), -0.55, 0.55)
        elif near_l:
            w -= 0.55 * (0.55 - sl) / 0.55
        elif near_r:
            w += 0.55 * (0.55 - sr) / 0.55
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
        """Force a turn back toward the lane centre near a side wall."""
        margin = ColorCourseNavigator.WALL_SIDE - 0.140
        if lat > margin:
            w = min(w, -0.60)
        elif lat < -margin:
            w = max(w, +0.60)
        return clamp(w, -ColorCourseNavigator.W_MAX,
                     ColorCourseNavigator.W_MAX)

    # ------------------------------------------------------- scan target
    def _choose_target(self, rel):
        """Most open corridor, biased toward the finish direction (abs 0).

        A candidate bearing is scored by the *worst* TOF range inside a
        +-SCAN_WINDOW rad corridor, not by its own ray alone.  Candidates are
        also restricted to a forward sector around the finish heading, so a
        long empty corridor behind the robot can never win.
        """
        if not self.scan:
            return 0.0
        win = max(1, int(round(self.SCAN_WINDOW / self.BIN)))
        best_b = None
        best_score = -1e9
        for b in self.scan:
            ang = b * self.BIN
            if abs(ang) > self.SWEEP_AMP + 0.01:
                continue
            heading = wrap(rel + ang)
            if abs(heading) > self.SCAN_FWD_LIMIT:
                continue
            vals = []
            blocked = False
            for k in range(-win, win + 1):
                rr = self.scan.get(b + k)
                if rr is None:
                    continue
                aa = (b + k) * self.BIN
                if self._is_side_wall(rr, aa) or self._is_wall(rr, aa):
                    blocked = True
                    break
                vals.append(rr)
            if blocked or not vals:
                continue
            score = (min(min(vals), self.MAX_RANGE)
                     - self.SCAN_BIAS * abs(heading))
            if score > best_score:
                best_score = score
                best_b = b
        if best_b is None:
            return 0.0
        return wrap(rel + best_b * self.BIN)

    # ------------------------------------------------------------- main entry
    def step(self, x, y, rel, t, sonar_left=INF, sonar_right=INF,
             red=0, green=0, finish=False, obs_x=0.0, obs_px=0):
        self.x, self.y, self.rel = x, y, rel
        if self.t_state == 0.0:
            self.t_state = t
        if red >= self.COLOR_PIX or green >= self.COLOR_PIX:
            self.last_color_t = t
        if finish:
            # The cyan strip is unique to the finish, so latching it is safe.
            self.finish_seen = True
            self.last_finish_t = t

        if self.state is None:
            self._enter('SCAN', t)
        if self.finished:
            return 0.0, 0.0

        s = self.state
        elapsed = t - self.t_state
        prog = self.progress(x, y)
        lat = self.lateral(x, y)

        if self.DEBUG and int(t * 2) != self._dbg:
            self._dbg = int(t * 2)
            print('[nav] %-10s p=%.2f lat=%+.2f rel=%+5.1fdeg red=%d green=%d '
                  'fin=%d sonar L=%.2f R=%.2f tgt=%+5.1fdeg'
                  % (s, prog, lat, math.degrees(rel), red, green, finish,
                     sonar_left, sonar_right, math.degrees(self.target_abs)),
                  flush=True)

        # ------------------------------------------------------- finish zone
        if s != 'FINAL':
            near_finish = prog >= self.LINE_MIN_P
            no_colour = (t - self.last_color_t) > 1.2
            if self.finish_seen or (near_finish and no_colour):
                print('[nav] finish zone -> FINAL', flush=True)
                self._enter('FINAL', t)
                s = 'FINAL'
                elapsed = 0.0

        # ------------------------------------------------------------------
        if s == 'SCAN':
            if elapsed < self.SCAN_TOTAL + 0.10:
                return 0.0, 0.0
            path = self._plan_path() if self.obstacles else None
            if path and len(path) >= 2:
                self.path = path
                self.path_i = 0
                print('[nav] scan -> plan %d waypoints toward finish'
                      % len(path), flush=True)
                self._enter('FOLLOW', t)
                return 0.0, 0.0
            self.target_abs = self._choose_target(rel)
            if self.scan:
                b = min(self.scan, key=lambda k: abs(wrap(k * self.BIN
                                                          - wrap(-rel))))
                print('[nav] scan: %d bearings, nearest-to-finish %.2f m at '
                      '%+.0f deg -> target %+.0f deg'
                      % (len(self.scan), self.scan[b],
                         math.degrees(b * self.BIN),
                         math.degrees(self.target_abs)), flush=True)
            else:
                print('[nav] scan: no returns -> hold finish heading', flush=True)
            self._enter('APPROACH', t)
            return 0.0, 0.0

        if s == 'FOLLOW':
            if not self.path:
                self._enter('SCAN', t)
                return 0.0, 0.0
            tgt, seg = self._path_target(x, y)
            self.path_i = seg
            d_end = math.hypot(self.path[-1][0] - x,
                               self.path[-1][1] - y)
            if d_end <= self.WP_TOL:
                print('[nav] plan complete -> final approach', flush=True)
                self._enter('FINAL', t)
                return 0.0, 0.0

            heading = math.atan2(tgt[1] - y, tgt[0] - x)
            err = wrap(heading - rel)
            v = self.V_FWD * (1.0 - min(0.75, abs(err) / 1.10))
            w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)

            # The known-layout plan already guarantees clearance, so the local
            # detector is only a last-ditch guard: if a box really is about to
            # touch the nose, stop and let the colour rule handle it.
            po = self._path_obstacle(t, half_width=0.060, max_r=0.20)
            if po is not None and po[2] <= 0.035:
                print('[nav] plan blocked -> colour check', flush=True)
                self._enter('CHECK', t)
                return 0.0, 0.0

            w = self._wall_guard(w, lat)
            w = self._wall_hard(w, lat)
            if min(sonar_left, sonar_right) < self.SIDE_STOP:
                v = 0.0
            elif min(sonar_left, sonar_right) < 0.060:
                v *= 0.65
            return v, w

        if s == 'APPROACH':
            fr = self._front(t)
            dc = fr[2] if fr is not None else INF
            po = self._path_obstacle(t)
            moved = math.hypot(x - self.sx, y - self.sy)
            side_min = min(sonar_left, sonar_right)

            # Escape when a box is scraping the body or the bot has stopped
            # making progress (the target heading and the side guard can
            # otherwise cancel each other out).
            if side_min < self.SIDE_REVERSE:
                if sonar_left <= sonar_right:
                    self.avoid_side = -1.0      # box on the left -> turn right
                else:
                    self.avoid_side = +1.0      # box on the right -> turn left
                print('[nav] boxed in (side=%.2f moved=%.2f) -> reverse away'
                      % (side_min, moved), flush=True)
                self._enter('AVOID', t)
                return 0.0, 0.0
            if elapsed > self.STUCK_T and moved < self.STUCK_DIST:
                if side_min < self.SIDE_HARD:
                    if sonar_left <= sonar_right:
                        self.avoid_side = -1.0
                    else:
                        self.avoid_side = +1.0
                    print('[nav] stuck (side=%.2f) -> reverse away'
                          % side_min, flush=True)
                    self._enter('AVOID', t)
                elif po is not None:
                    print('[nav] stuck behind a box -> check colour',
                          flush=True)
                    self._enter('CHECK', t)
                else:
                    print('[nav] no progress -> re-scan', flush=True)
                    self._enter('SCAN', t)
                return 0.0, 0.0

            if dc <= self.CHECK_DIST \
                    or (po is not None and po[2] <= self.CHECK_DIST):
                self._enter('CHECK', t)
                return 0.0, 0.0
            if elapsed > self.T_APPROACH:
                print('[nav] approach timeout -> re-scan', flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0

            # If a detour has left the bot facing away from the finish, turn
            # in place first instead of driving backwards across the arena.
            face = wrap(0.0 - rel)
            if abs(face) > math.radians(80):
                w = clamp(2.2 * face, -self.TURN_W_MAX, self.TURN_W_MAX)
                return 0.0, w

            if po is not None:
                # A box inside the body's swept corridor: centre it before the
                # camera check so the sidestep is measured from a true head-on
                # approach.
                w = clamp(2.0 * po[0], -self.W_MAX, self.W_MAX)
                v = self.V_FWD * max(0.30, po[2] / 0.60)
            else:
                err = wrap(self.target_abs - rel)
                w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)
                if obs_px >= 120 and abs(obs_x) > 0.02:
                    w = clamp(-self.K_CAM * obs_x, -self.W_MAX, self.W_MAX)
                v = self.V_FWD

            # Hard override: very close side obstacle beats the target
            # heading, so the body always steers toward the freer side.
            if side_min < self.SIDE_HARD:
                if sonar_left < sonar_right:
                    w = -0.7 * self.W_MAX
                else:
                    w = +0.7 * self.W_MAX

            w = self._side_guard(w, sonar_left, sonar_right)
            w = self._wall_guard(w, lat)
            w = self._wall_hard(w, lat)

            if dc < 0.40:
                v *= max(0.30, (dc - 0.06) / 0.34)
            if abs(lat) > self.WALL_SIDE - 0.140:
                v *= 0.50
            if side_min < self.SIDE_STOP:
                v = 0.0
            elif side_min < 0.120:
                v *= 0.65
            return v, w

        if s == 'AVOID':
            # Rotate in place toward the open side, then let a fresh scan pick
            # the next corridor.  No blind reversing: there is no rear sensor.
            if elapsed >= self.T_AVOID:
                print('[nav] escape done -> re-scan', flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0
            w = self._wall_hard(self.avoid_side * self.W_AVOID, lat)
            return 0.0, w

        if s == 'CHECK':
            self.check_red = max(self.check_red, red)
            self.check_green = max(self.check_green, green)
            if elapsed < self.COLOR_WAIT:
                return 0.0, 0.0
            if max(self.check_red, self.check_green) < self.COLOR_PIX:
                print('[nav] no colour in front -> re-scan', flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0

            if self.check_red >= self.check_green:
                color, side = 'red', +1
            else:
                color, side = 'green', -1

            # Keep the colour rule unless the preferred side has no room: a
            # small arena means the detour must not drive into a wall.
            room_l = self.WALL_SIDE - lat
            room_r = self.WALL_SIDE + lat
            if side > 0 and room_l < 0.200 and room_r > room_l:
                print('[nav] left side blocked by wall -> mirror the detour',
                      flush=True)
                side = -1
            elif side < 0 and room_r < 0.200 and room_l > room_r:
                print('[nav] right side blocked by wall -> mirror the detour',
                      flush=True)
                side = +1

            # How far to sidestep: account for the robot already being
            # offset from the box, so a box sitting on the detour side still
            # ends up fully cleared.
            src = self._path_obstacle(t) or self._front(t, 0.85)
            if obs_px >= self.COLOR_PIX:
                # Camera centroid is a body-frame lateral offset; rotate it
                # into the fixed start frame using the current heading.
                cam_lat = clamp(-obs_x * 0.14, -0.16, 0.16)
                cam_fwd = src[1] if src is not None else 0.20
                box_rel = cam_fwd * math.sin(rel) + cam_lat * math.cos(rel)
            elif src is not None:
                box_rel = src[1] * math.sin(rel + src[0])
            else:
                box_rel = 0.0
            self.box_rel = clamp(box_rel, -0.22, 0.22)
            self.side_need = clamp(side * self.box_rel + self.SIDE_CLEAR_BOX,
                                   self.PASS_CLEAR_MIN, self.PASS_LAT_MAX)
            self.side_flipped = False
            if self.DEBUG:
                print('[nav] check: obs_x=%+.2f px=%d box_rel=%+.2f -> need %.2f'
                      % (obs_x, obs_px, self.box_rel, self.side_need),
                      flush=True)

            self.maneuvers += 1
            self.pass_color = color
            self.pass_side = side
            self.approach_heading = rel
            self.pass_heading = wrap(rel + side * math.pi / 2.0)
            print('[nav] box #%d is %s (red=%d green=%d) -> turn %s, sidestep '
                  '%.2f m, cross, straighten'
                  % (self.maneuvers, color.upper(), self.check_red,
                     self.check_green, 'LEFT' if side > 0 else 'RIGHT',
                     self.side_need),
                  flush=True)
            self._enter('TURN_SIDE', t)
            return 0.0, 0.0

        if s == 'TURN_SIDE':
            w, done = self._turn_to(self.pass_heading, t)
            if done or elapsed > self.T_TURN:
                self.side_seen = False
                self._enter('SIDE_MOVE', t)
                return 0.0, 0.0
            return 0.0, w

        if s == 'SIDE_MOVE':
            # Dead-reckoned lateral progress: pass_side +1 moves left (+lat),
            # -1 moves right (-lat).  Arena room is known from the pose, so we
            # do not depend on a side sonar that has just been rotated 90 deg.
            dlat = (lat - self.lat_entry) * self.pass_side
            room = (self.WALL_SIDE * self.pass_side - lat) * self.pass_side
            fr = self._front(t, 0.45)
            ahead = fr is not None and fr[2] <= self.CHECK_DIST + 0.06
            remaining = self.side_need - dlat
            need_mirror = (ahead and remaining > 0.0
                           and fr[2] - 0.100 < remaining)
            # Only give up on this side if the neighbouring box is actually
            # closer than the remaining sidestep.
            if need_mirror:
                # The chosen side is blocked by another box before the target
                # is cleared.  Mirror the detour once; otherwise re-plan.
                other = -self.pass_side
                room_other = (self.WALL_SIDE * other - lat) * other
                if not self.side_flipped and room_other > 0.180:
                    self.side_flipped = True
                    self.pass_side = other
                    self.side_need = clamp(
                        other * self.box_rel + self.SIDE_CLEAR_BOX,
                        self.PASS_CLEAR_MIN, self.PASS_LAT_MAX)
                    self.pass_heading = wrap(
                        self.approach_heading + other * math.pi / 2.0)
                    print('[nav] sidestep blocked -> mirror to %s (need %.2f)'
                          % ('LEFT' if other > 0 else 'RIGHT',
                             self.side_need), flush=True)
                    self._enter('TURN_SIDE', t)
                    return 0.0, 0.0
                print('[nav] both sides blocked -> re-scan', flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0
            done = dlat >= self.side_need \
                or room <= 0.150 \
                or elapsed > self.T_SIDE
            if done:
                print('[nav] sidestep done: dlat=%+.2f need=%.2f room=%.2f'
                      % (dlat, self.side_need, room), flush=True)
                self._enter('TURN_BACK', t)
                return 0.0, 0.0
            v, w = self._drive_heading(self.pass_heading, self.V_PASS)
            return v, self._wall_hard(clamp(w, -self.W_MAX, self.W_MAX), lat)

        if s == 'TURN_BACK':
            # Straighten onto the finish heading (IMU), then cross.
            w, done = self._turn_to(0.0, t)
            if done or elapsed > self.T_TURN:
                self._enter('CROSS', t)
                return 0.0, 0.0
            return 0.0, w

        if s == 'CROSS':
            moved = math.hypot(x - self.sx, y - self.sy)
            if moved >= self.CROSS_MIN or elapsed > self.CROSS_MAX_T:
                print('[nav] crossed (moved=%.2f) -> re-scan' % moved,
                      flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0
            fr = self._front(t, 0.45)
            if fr is not None and fr[2] <= self.CHECK_DIST + 0.06:
                print('[nav] new box directly ahead -> re-scan', flush=True)
                self._enter('SCAN', t)
                return 0.0, 0.0
            v, w = self._drive_heading(0.0, self.V_PASS)
            # Actively keep away from the box being passed: its side sonar
            # will see it during the cross.
            w = self._side_guard(w, sonar_left, sonar_right)
            w = self._wall_guard(w, lat)
            w = self._wall_hard(w, lat)
            return v, w

        if s == 'FINAL':
            herr = wrap(0.0 - rel)
            if abs(herr) > math.radians(8):
                w, _ = self._turn_to(0.0, t)
                return 0.0, w

            if prog >= self.FINISH_P:
                self.finished = True
                self.finish_reason = ('finish line colour detected'
                                      if self.finish_seen
                                      else 'reached finish line')
                print('[nav] ON THE FINISH LINE (progress=%.2f m)' % prog,
                      flush=True)
                return 0.0, 0.0

            raw = self._raw_front(t, 0.40)
            if raw is not None and raw[2] < self.WALL_STOP \
                    and self._is_wall(raw[1], raw[0]):
                self.finished = True
                if self.finish_seen:
                    self.finish_reason = ('finish line crossed '
                                          '(safe stop before wall)')
                else:
                    self.finish_reason = 'end-wall safety stop'
                print('[nav] end-wall safety stop (range=%.2f m)' % raw[2],
                      flush=True)
                return 0.0, 0.0

            if elapsed > self.T_FINAL:
                self.finished = True
                self.finish_reason = 'final-approach timeout'
                print('[nav] FINAL timeout -> safe stop', flush=True)
                return 0.0, 0.0

            v = self.V_FINAL
            if raw is not None and raw[2] < 0.35:
                v *= max(0.40, (raw[2] - 0.08) / 0.27)
            return v, clamp(self.K_HEAD * herr, -0.25, 0.25)

        return 0.0, 0.0

    # ---------------------------------------------------------------- helpers
    def _enter(self, state, t):
        self.state = state
        self.t_state = t
        self.ok_since = None
        self.sx, self.sy = self.x, self.y
        self.lat_entry = self.lateral(self.x, self.y)
        if state == 'SCAN':
            self.scan_t0 = t
            self.scan_active = True
            self.scan = {}
        elif state == 'CHECK':
            self.check_red = self.check_green = 0
