"""Colour-course navigator for the autonomous bot (pure Python, no ROS).

The arena contains ten red/green boxes arranged as five colour gates (two
boxes per row).  Every gate has one box directly on the centre line and one
box on the open/closed side, so the robot can always aim at the centre box:

    SCAN      sweep the servo/TOF once, build a polar map, report the best
              forward bearing toward the course
    APPROACH  drive at the centred gate box; far away the TOF bearing steers,
              close up a pure-pursuit lane hold lines the box up with the body
    CHECK     stop just before the box, straighten, and read its colour from
              the angled camera (lower image rows only, so the next gate
              cannot win)
    red       turn LEFT  -> sidestep left  -> turn back -> cross the gate
    green     turn RIGHT -> sidestep right -> turn back -> cross the gate
    RECENTER  curve back onto the lane centre line, straighten in place, then
              look for the next centred box; repeat until all five are passed
    FINAL     finish-line colour detected (or end wall reached): stop safely
              and report the total challenge time

Angles are relative to the start heading: 0 = toward the finish, + = left.
Only simple sensor values are consumed, so the class is unit-testable.
"""

import math

INF = 9.9


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ColorCourseNavigator:
    # ---------------------------------------------------------------- arena
    # Start is y = -1.35, finish line y = +1.30, end-wall inner face y = +1.475.
    # The arena holds five compact gates (ten boxes) with ~0.57 m row spacing.
    GATES         = 5
    WALL_P        = 2.825       # end wall inner face, from start position
    WALL_SIDE     = 0.475       # side wall inner face, from the lane centre
    WALL_STOP     = 0.120       # never get closer than this (beam -> wall)
    BEAM_X        = 0.100       # TOF beam pivot ahead of the axle
    LANE_HALF     = 0.345       # keep the axle inside this lateral band
    FINISH_P      = 2.620       # axle progress on the finish line
    LINE_MIN_P    = 2.520       # ignore the cyan strip before this progress

    # ---------------------------------------------------------------- scan
    SWEEP_AMP     = 1.05        # servo sweep, rad each side (one-time map)
    BIN           = 0.05        # polar map resolution, rad
    SCAN_SETTLE   = 0.50        # let the servo reach the left limit
    SCAN_SWEEP    = 3.00        # left -> right sweep time (slow = dense map)
    SCAN_RETURN   = 0.50        # return the head to straight ahead
    SCAN_TOTAL    = SCAN_SETTLE + SCAN_SWEEP + SCAN_RETURN
    MAX_RANGE     = 3.0
    # After the one-time scan the head keeps sweeping gently so the TOF keeps
    # giving a BEARING to the obstacle, not just a distance straight ahead.
    APPROACH_AMP  = 0.55        # rad each side while driving to a gate
    APPROACH_PER  = 1.40        # seconds for a full left->right->left sweep

    # -------------------------------------------------------------- turning
    K_TURN        = 2.2
    TURN_W_MAX    = 0.90
    TURN_W_MIN    = 0.22
    ALIGN_TOL     = math.radians(3)
    SETTLE        = 0.18

    # -------------------------------------------------------------- driving
    V_FWD         = 0.120
    V_PASS        = 0.075
    V_FINAL       = 0.060
    K_HEAD        = 2.8
    K_LAT         = 3.0
    K_CAM         = 2.0         # camera-centroid steering gain
    W_MAX         = 0.85

    # ----------------------------------------------------- obstacle / colour
    CHECK_DIST    = 0.060       # forward TOF distance at which to stop
    COLOR_PIX     = 30          # colour pixels that count as "obstacle seen"
    COLOR_WAIT    = 0.55        # camera sampling time while stopped

    # ---------------------------------------------------------- side sonars
    SIDE_GUARD    = 0.160       # start steering away from a side obstacle
    SIDE_STOP     = 0.030       # hard stop: body would nearly touch it
    SIDE_WALL_STOP = 0.045      # stop sidestep before the side wall
    PASS_LAT      = 0.220       # sidestep distance for the colour rule
    PASS_LAT_MIN  = 0.120       # still a clear pass when the wall is close

    # ------------------------------------------------------------- crossing
    CROSS_MIN      = 0.300      # forward travel that decides the gate is clear
    CROSS_MAX      = 0.390      # hard upper bound for the crossing leg
    RECENTER_DIST  = 0.180      # forward distance allowed to re-centre
    RECENTER_TOL   = 0.055      # lateral error that counts as re-centred
    CROSS_MAX_T    = 8.0

    # -------------------------------------------------------------- timeouts
    T_SCAN        = 1.0
    T_APPROACH    = 12.0
    T_TURN        = 5.0
    T_SIDE        = 5.0
    T_BACK        = 4.0
    T_CHECK       = 2.0

    # --------------------------------------------------------------- debug
    DEBUG         = True

    def __init__(self):
        self.reset(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ init
    def reset(self, x0, y0, yaw0):
        self.x0, self.y0, self.yaw0 = x0, y0, yaw0
        self.state = None
        self.t_state = 0.0
        self.t0 = None              # absolute time of START (servo timeline)
        self.ok_since = None
        self.sx = self.sy = 0.0
        self.x, self.y, self.rel = x0, y0, 0.0

        self.finished = False
        self.finish_reason = None
        self.finish_seen = False
        self.hits = 0               # gate pairs passed
        self.check_retry = 0

        self.target = 0.0
        self.scan = {}              # servo bearing bin -> shortest range
        self.recent = []            # (t, bearing, range) for the front veto

        self.pass_color = None
        self.pass_side = 0
        self.pass_lat = self.PASS_LAT
        self.approach_heading = 0.0
        self.pass_heading = 0.0
        self.side_seen = False
        self.side_d0 = INF
        self.gate_seen = False
        self.gate_d0 = INF
        self.check_red = 0
        self.check_green = 0
        self.backup_then = 'APPROACH'
        self.turn_then = 'CROSS'
        self.last_color_t = -1e9
        self.last_finish_t = -1e9
        self._dbg = -1

    # ---------------------------------------------------------- pose helpers
    def progress(self, x, y):
        return ((x - self.x0) * math.cos(self.yaw0)
                + (y - self.y0) * math.sin(self.yaw0))

    def lateral(self, x, y):
        """Signed distance from the start line: + = left of the start heading."""
        return (-(x - self.x0) * math.sin(self.yaw0)
                + (y - self.y0) * math.cos(self.yaw0))

    # ------------------------------------------------------------- servo/TOF
    def servo_cmd(self, t):
        """One slow sweep before the first approach, then a gentle sweep.

        The continued sweep keeps giving the TOF a BEARING to the box rather
        than only a straight-ahead range.
        """
        if self.t0 is None:
            self.t0 = t
            self.t_state = t
        e = t - self.t0
        if e < self.SCAN_SETTLE:
            return -self.SWEEP_AMP
        e -= self.SCAN_SETTLE
        if e < self.SCAN_SWEEP:
            return -self.SWEEP_AMP + 2.0 * self.SWEEP_AMP * (e / self.SCAN_SWEEP)
        e -= self.SCAN_SWEEP
        if e < self.SCAN_RETURN:
            return self.SWEEP_AMP * (1.0 - e / self.SCAN_RETURN)
        # Keep sweeping after the one-time scan: the extra bearings let the
        # navigator steer toward a box that is not exactly dead ahead.  Start
        # the sweep at 0 rad (sine) so the first APPROACH step looks straight
        # at the near gate instead of at the far next gate.
        e -= self.SCAN_RETURN
        return self.APPROACH_AMP * math.sin(
            2.0 * math.pi * e / self.APPROACH_PER)

    def tof(self, bearing, rng, t):
        """Feed one TOF sample: servo joint angle (rad) and range (m)."""
        if not math.isfinite(rng):
            return
        self.recent.append((t, bearing, rng))
        if len(self.recent) > 160:
            del self.recent[:-160]
        if self.t0 is not None and t - self.t0 <= self.SCAN_TOTAL + 0.10:
            if abs(bearing) <= self.SWEEP_AMP + 0.06:
                b = int(round(bearing / self.BIN))
                if b not in self.scan or rng < self.scan[b]:
                    self.scan[b] = rng

    # ------------------------------------------------------------- wall test
    def _is_wall(self, d, bearing):
        # Beam direction relative to the LANE (not the body) for the wall maths.
        c = math.cos(self.rel + bearing)
        if c < 0.3:
            return False
        p_b = self.progress(self.x, self.y) + self.BEAM_X * math.cos(self.rel)
        expected = (self.WALL_P - p_b) / c
        # only trust the wall test close to the end wall; odometry drifts
        if expected > 1.2:
            return False
        return d >= expected - 0.18

    def _is_side_wall(self, d, bearing):
        """True when a return is really the left/right arena wall.

        The servo sweeps wider than the gate box, so at +-0.5 rad the narrow
        TOF can hit the side wall ~0.8-1.1 m away.  Without this test the
        navigator would treat that wall as an obstacle bearing and swerve
        violently; the walls are not obstacles to steer around here.
        """
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

    def _front(self, t, cone=1.00):
        """Nearest fresh TOF return inside the swept forward cone.

        Returns (bearing, range, forward_distance).  The one-time scan is
        followed by a gentle continuous sweep, so a box that is off to one
        side still produces a bearing that the navigator can steer towards.
        Forward distance uses the beam angle so the stop threshold does not
        depend on where in the sweep the box was seen.

        The default stop cone is deliberately wide: as the robot closes on a
        box a small lateral offset becomes a large bearing, and a narrow cone
        used to lose the box just before the stop.  Arena walls are removed by
        _is_wall()/_is_side_wall(), so they cannot win this test.
        """
        vals = []
        for tt, a, r in reversed(self.recent):
            if t - tt > 0.90:
                break
            if abs(a) <= cone and not self._is_wall(r, a) \
                    and not self._is_side_wall(r, a):
                vals.append((a, r))
            if len(vals) >= 24:
                break
        if not vals:
            return None
        a, r = min(vals, key=lambda v: v[1] * math.cos(v[0]))
        return a, r, r * math.cos(a)

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

    def _drive_heading(self, target, v_max=None, lat=0.0, center=True):
        err = wrap(target - self.rel)
        v = self.V_FWD if v_max is None else v_max
        v *= 1.0 - min(0.85, abs(err) / 0.6)
        w = clamp(self.K_HEAD * err, -self.W_MAX, self.W_MAX)
        if center:
            # steer back toward the lane centre line as well
            w -= clamp(self.K_LAT * lat, -0.55, 0.55)
        return v, clamp(w, -self.W_MAX, self.W_MAX)

    def _side_guard(self, w, sl, sr):
        if sl < self.SIDE_GUARD:
            w -= 0.8 * (self.SIDE_GUARD - sl) / self.SIDE_GUARD
        if sr < self.SIDE_GUARD:
            w += 0.8 * (self.SIDE_GUARD - sr) / self.SIDE_GUARD
        return clamp(w, -self.W_MAX, self.W_MAX)

    # ------------------------------------------------------------- main entry
    def step(self, x, y, rel, t, sonar_left=INF, sonar_right=INF,
             red=0, green=0, finish=False, obs_x=0.0, obs_px=0):
        self.x, self.y, self.rel = x, y, rel
        if self.t0 is None:
            self.t0 = t
            self.t_state = t
        if red >= self.COLOR_PIX or green >= self.COLOR_PIX:
            self.last_color_t = t
        if finish and self.hits >= self.GATES - 1:
            # Latch cyan only once the robot is at the last gate; the strip
            # lies just beyond it and can be glimpsed during the final cross.
            self.last_finish_t = t
            self.finish_seen = True

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
                  'fin=%d sonar L=%.2f R=%.2f'
                  % (s, prog, lat, math.degrees(rel), red, green, finish,
                     sonar_left, sonar_right), flush=True)

        # ------------------------------------------------- finish the course
        # All five gates handled: creep out of the last gate and stop on the
        # cyan strip (or on the end-wall guard if the colour is not visible).
        if s not in ('FINAL', 'TURN_SIDE', 'SIDE_MOVE', 'TURN_BACK', 'CROSS'):
            if self.hits >= self.GATES and (finish or prog >= self.LINE_MIN_P):
                self._enter('FINAL', t)
                s = 'FINAL'

        # ------------------------------------------------------------------
        if s == 'SCAN':
            if elapsed < self.SCAN_TOTAL and (t - self.t0) < self.SCAN_TOTAL:
                return 0.0, 0.0
            if self.scan:
                b = min(self.scan, key=lambda k: abs(k * self.BIN))
                print('[nav] scan complete: %d bearings, closest forward '
                      'return %.2f m at %+.0f deg'
                      % (len(self.scan), self.scan[b], math.degrees(b * self.BIN)),
                      flush=True)
                if self.DEBUG:
                    print('[nav] polar map: %s'
                          % {round(k * self.BIN, 2): round(v, 2)
                             for k, v in sorted(self.scan.items())}, flush=True)
            else:
                print('[nav] scan complete: no forward return', flush=True)
            self._enter('APPROACH', t)
            return 0.0, 0.0

        if s == 'APPROACH':
            fr = self._front(t)                  # wide: for the stop distance
            dc = fr[2] if fr is not None else INF
            if dc <= self.CHECK_DIST:
                self.check_red = self.check_green = 0
                self._enter('CHECK', t)
                return 0.0, 0.0
            if elapsed > self.T_APPROACH:
                print('[nav] approach timeout -> re-scan', flush=True)
                self._enter('SCAN', t)
                self.t0 = t              # second scan (also a fresh servo sweep)
                self.scan = {}
                return 0.0, 0.0

            # Steering priority: a close TOF return (the gate box), then a
            # strong camera colour blob, then the lane centre line.  A far
            # TOF return (the next gate) must not win while the camera still
            # sees the near box, otherwise the robot drifts off the gate.
            fs = self._front(t, 0.45)
            if fs is not None and 0.35 <= fs[2] < 0.45:
                w = clamp(2.2 * fs[0], -self.W_MAX, self.W_MAX)
            elif obs_px >= 120 and abs(obs_x) > 0.02:
                w = clamp(-self.K_CAM * obs_x, -self.W_MAX, self.W_MAX)
            else:
                # Pure-pursuit to the lane centre: includes heading damping,
                # so a correction cannot turn into a slow spiral.
                ld = 0.16
                err = wrap(math.atan2(-lat, ld) - rel)
                w = clamp(3.0 * err, -self.W_MAX, self.W_MAX)
            w = self._side_guard(w, sonar_left, sonar_right)

            v = self.V_FWD
            if dc < 0.40:
                v *= max(0.30, (dc - 0.06) / 0.34)
            # A side sonar reading very close while driving straight is the
            # gate box we are intentionally passing, not a frontal collision.
            # _side_guard() above already steers away from it gently.
            return v, w

        if s == 'CHECK':
            # Straighten first: the sidestep must be perpendicular to the lane,
            # otherwise it drifts forward/backward and the clearance changes.
            if abs(rel) > math.radians(6):
                w, _ = self._turn_to(0.0, t)
                return 0.0, w
            self.check_red = max(self.check_red, red)
            self.check_green = max(self.check_green, green)
            if elapsed < self.COLOR_WAIT:
                return 0.0, 0.0
            if max(self.check_red, self.check_green) < self.COLOR_PIX:
                self.check_retry += 1
                print('[nav] no obstacle colour in front (retry %d)' %
                      self.check_retry, flush=True)
                if self.check_retry > 2:
                    print('[nav] colour not readable -> skip to finish test',
                          flush=True)
                    self._enter('FINAL', t)
                else:
                    self._enter('BACKUP', t)
                    self.backup_then = 'APPROACH'
                return 0.0, 0.0

            self.check_retry = 0
            if self.check_red >= self.check_green:
                color, side = 'red', +1
            else:
                color, side = 'green', -1

            # Never break the red-left / green-right rule; only shorten the
            # sidestep when the wall on that side is unusually close.
            room = (self.WALL_SIDE - lat) if side > 0 else (self.WALL_SIDE + lat)
            self.pass_lat = clamp(min(self.PASS_LAT, room - 0.070),
                                  self.PASS_LAT_MIN, self.PASS_LAT)
            if self.pass_lat < self.PASS_LAT - 0.005:
                print('[nav] wall close: sidestep shortened to %.2f m'
                      % self.pass_lat, flush=True)

            self.pass_color = color
            self.pass_side = side
            self.approach_heading = self.rel
            # Absolute lane-frame heading: a true sideways sidestep.
            self.pass_heading = wrap(side * math.pi / 2.0)
            self.hits += 1
            print('[nav] obstacle pair #%d is %s (red=%d green=%d) -> turn %s, '
                  'sidestep, cross'
                  % (self.hits, color.upper(), self.check_red,
                     self.check_green, 'LEFT' if side > 0 else 'RIGHT'),
                  flush=True)
            self._enter('TURN_SIDE', t)
            return 0.0, 0.0

        if s == 'TURN_SIDE':
            w, done = self._turn_to(self.pass_heading, t)
            if done or elapsed > self.T_TURN:
                self.sx, self.sy = x, y
                self._enter('SIDE_MOVE', t)
                return 0.0, 0.0
            return 0.0, w

        if s == 'SIDE_MOVE':
            moved = math.hypot(x - self.sx, y - self.sy)
            target = self.pass_side * self.pass_lat
            # Absolute lateral target, not a relative distance: an off-centre
            # approach must still end at the same pass lane.
            if self.pass_side > 0:
                reached = lat >= target - 0.012
            else:
                reached = lat <= target + 0.012
            side_d = sonar_right if self.pass_side > 0 else sonar_left
            wall_d = sonar_left if self.pass_side > 0 else sonar_right
            done = (reached or wall_d < self.SIDE_WALL_STOP
                    or elapsed > self.T_SIDE)
            if done:
                print('[nav] sidestep done: moved=%.2f lat=%+.2f target=%+.2f '
                      'sonar=%.2f' % (moved, lat, target, side_d), flush=True)
                self.turn_then = 'CROSS'
                self._enter('TURN_BACK', t)
                return 0.0, 0.0
            v, w = self._drive_heading(self.pass_heading, self.V_PASS,
                                       center=False)
            if wall_d < self.SIDE_GUARD * 0.6:
                w += self.pass_side * 0.30
            return v, clamp(w, -self.W_MAX, self.W_MAX)

        if s == 'TURN_BACK':
            w, done = self._turn_to(0.0, t)
            if done or elapsed > self.T_TURN:
                self.sx, self.sy = x, y
                self._enter(self.turn_then, t)
                return 0.0, 0.0
            return 0.0, w

        if s == 'CROSS':
            moved = math.hypot(x - self.sx, y - self.sy)
            if moved >= self.CROSS_MIN or moved >= self.CROSS_MAX \
                    or elapsed > self.CROSS_MAX_T:
                print('[nav] gate %d crossed (moved=%.2f)' %
                      (self.hits, moved), flush=True)
                self.sx, self.sy = x, y
                if self.hits >= self.GATES:
                    # Last gate cleared: the cyan strip is just ahead.
                    self._enter('FINAL', t)
                else:
                    self._enter('RECENTER', t)
                return 0.0, 0.0
            if min(sonar_left, sonar_right) < self.SIDE_STOP:
                # Only the OPEN side matters while crossing: the boxes we are
                # passing sit on the closed side and legitimately read ~0.03 m.
                open_d = sonar_left if self.pass_side > 0 else sonar_right
                if open_d < self.SIDE_STOP:
                    print('[nav] side obstacle during cross L=%.2f R=%.2f'
                          % (sonar_left, sonar_right), flush=True)
                    self._enter('BACKUP', t)
                    self.backup_then = 'CROSS'
                    return 0.0, 0.0
            # Hold the heading; do NOT side-steer here.  A guard reaction
            # while passing a box only pushes the robot off the lane.
            v, w = self._drive_heading(0.0, self.V_PASS, center=False)
            return v, w

        if s == 'RECENTER':
            moved = math.hypot(x - self.sx, y - self.sy)
            if abs(lat) < self.RECENTER_TOL or moved >= self.RECENTER_DIST \
                    or elapsed > 3.5:
                # Stop here and straighten in place before approaching the
                # next gate: an angled approach can put the box outside the
                # servo sweep, which used to trigger a timeout spin.
                self.turn_then = 'APPROACH'
                self._enter('TURN_BACK', t)
                return 0.0, 0.0
            # Pure-pursuit back to a look-ahead point on the lane centre.
            ld = 0.14
            desired = math.atan2(-lat, ld)
            err = wrap(desired - rel)
            w = clamp(3.0 * err, -self.W_MAX, self.W_MAX)
            return self.V_PASS, w

        if s == 'BACKUP':
            moved = math.hypot(x - self.sx, y - self.sy)
            if moved >= 0.10 or elapsed > 1.5:
                self._enter(self.backup_then, t)
                return 0.0, 0.0
            return -0.06, clamp(self.K_HEAD * wrap(0.0 - rel),
                                -0.4, 0.4)

        if s == 'FINAL':
            herr = wrap(0.0 - rel)
            if abs(herr) > math.radians(10):
                w, _ = self._turn_to(0.0, t)
                return 0.0, w

            # 1. The cyan strip was already detected (latched as soon as the
            #    last gate was reached).  Keeping the latch avoids losing the
            #    detection while the robot straightens after the final cross.
            if self.finish_seen or (finish and prog >= self.LINE_MIN_P):
                self.finished = True
                self.finish_reason = 'finish line colour detected'
                print('[nav] FINISH LINE COLOUR DETECTED -> stop', flush=True)
                return 0.0, 0.0

            # 2. Raw front-wall guard: always stop short of the end wall even
            #    if the cyan strip is dirty / out of view.
            dc = self._center(t)
            if dc < self.WALL_STOP and self._is_wall(dc, rel):
                self.finished = True
                self.finish_reason = 'end-wall safety stop'
                print('[nav] end-wall safety stop (range=%.2f m)' % dc,
                      flush=True)
                return 0.0, 0.0

            # 3. Last-resort progress check (dead reckoned, so it no longer
            #    runs ahead/behind after the many in-place turns).
            if prog >= self.FINISH_P:
                self.finished = True
                self.finish_reason = 'reached finish line'
                print('[nav] ON THE FINISH LINE (progress=%.2f m)' % prog,
                      flush=True)
                return 0.0, 0.0
            if elapsed > 12.0:
                self.finished = True
                self.finish_reason = 'final-approach timeout'
                print('[nav] FINAL timeout -> safe stop', flush=True)
                return 0.0, 0.0
            return self.V_FINAL, clamp(self.K_HEAD * herr, -0.25, 0.25)

        return 0.0, 0.0

    # ---------------------------------------------------------------- helpers
    def _enter(self, state, t):
        self.state = state
        self.t_state = t
        self.ok_since = None
        self.sx, self.sy = self.x, self.y
