#!/usr/bin/env python3
"""
apple_toss.py  --  Unitree G1 (Menagerie, with Dex3 hands) tosses an apple from one BARE hand
to the other BARE hand in MuJoCo.  No trays, no extra grippers, no teleporting.

Only existing G1 joints / actuators are used (7 arm joints + 7 hand joints per side).
The apple is a free rigid body (free joint) that is only ever moved by gravity and contacts.

Run:   python apple_toss.py                 (interactive viewer)
       python apple_toss.py --headless      (no window, prints a report)
"""
import argparse, os, sys, time
import numpy as np
import mujoco

# ----------------------------------------------------------------------------------------------
# 0.  USER SETTINGS
# ----------------------------------------------------------------------------------------------
# Folder that contains g1_with_hands.xml / scene_with_hands.xml (MuJoCo Menagerie -> unitree_g1)
DEFAULT_G1_DIR = os.path.expanduser("~/mujoco_menagerie/unitree_g1")
SCENE_NAME     = "scene_apple_toss.xml"      # scene = your G1 scene + the apple body (auto-written)

SCENE_XML = """<mujoco model="g1_with_hands apple toss scene">
  <include file="scene_with_hands.xml"/>
  <worldbody>
    <body name="apple" pos="0.3 0.17 1.2">
      <freejoint name="apple_free"/>
      <geom name="apple_geom" type="sphere" size="0.035" mass="0.15"
            friction="1.0 0.01 0.015" condim="6" solref="0.006 1" solimp="0.95 0.99 0.001"
            rgba="0.85 0.08 0.08 1"/>
      <geom name="apple_stem" type="capsule" fromto="0 0 0.030 0 0 0.046" size="0.003"
            contype="0" conaffinity="0" mass="0" rgba="0.35 0.2 0.05 1"/>
    </body>
  </worldbody>
</mujoco>
"""

G = 9.81
SIDES = ("left", "right")
SGN = {"left": 1.0, "right": -1.0}            # +y is the robot's left
ARM_JOINTS  = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
               "wrist_roll", "wrist_pitch", "wrist_yaw"]
HAND_JOINTS = ["hand_thumb_0", "hand_thumb_1", "hand_thumb_2",
               "hand_middle_0", "hand_middle_1", "hand_index_0", "hand_index_1"]

# Finger configurations, written for the LEFT hand (right hand = mirrored = negated).
#              thumb0 thumb1 thumb2 mid0  mid1  idx0  idx1
FING_CRADLE = np.array([0.0, -0.70, 0.0, -1.2, -1.0, -1.2, -1.0])  # thumb = back stop, fingers curl up
FING_OPEN   = np.array([0.0, -0.70, 0.0,  0.0,  0.0,  0.0,  0.0])  # flat palm, thumb tucked back
FING_CATCH  = np.array([0.0, -0.70, 0.0, -0.5, -0.4, -0.5, -0.4])  # catcher: shallow bowl waiting for the apple

# ----------------------------------------------------------------------------------------------
# 1.  small math helpers
# ----------------------------------------------------------------------------------------------
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def so3_log(R):
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(th))
    return w * th

def so3_exp(w):
    th = np.linalg.norm(w)
    if th < 1e-9:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K

def minjerk(t):   t = np.clip(t, 0, 1); return 10*t**3 - 15*t**4 + 6*t**5

def quintic(p0, v0, a0, p1, v1, a1, T):
    """Coefficients of the quintic polynomial matching position/velocity/acceleration at 0 and T."""
    A = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 2, 0, 0, 0],
                  [1, T, T**2, T**3, T**4, T**5], [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                  [0, 0, 2, 6*T, 12*T**2, 20*T**3]], float)
    b = np.array([p0, v0, a0, p1, v1, a1], float)
    return np.linalg.solve(A, b)

def smooth(Q, w=3):
    pad = w // 2
    Qp = np.pad(Q, ((pad, pad), (0, 0)), mode="edge"); k = np.ones(w) / w
    return np.stack([np.convolve(Qp[:, j], k, mode="valid") for j in range(Q.shape[1])], 1)

class JointTraj:
    """Joint-space reference q(t), qd(t), qdd(t), sampled at fixed dt and linearly interpolated."""
    def __init__(self, t0, Q, dt):
        self.t0, self.dt = t0, dt
        self.Q = smooth(Q, 3)
        self.QD = np.gradient(self.Q, dt, axis=0)
        self.QDD = np.gradient(smooth(self.QD, 3), dt, axis=0)
        self.T = dt * (len(Q) - 1)
    def at(self, t):
        x = np.clip((t - self.t0) / self.dt, 0, len(self.Q) - 1 - 1e-9)
        i = int(x); a = x - i
        f = lambda A: A[i] * (1 - a) + A[i + 1] * a
        inside = (t - self.t0) < self.T
        return f(self.Q), (f(self.QD) if inside else 0 * self.Q[0]), (f(self.QDD) if inside else 0 * self.Q[0])

# ----------------------------------------------------------------------------------------------
# 2.  the demo controller
# ----------------------------------------------------------------------------------------------
class AppleToss:
    # ---- tunable throw parameters (G1 workspace is small: hands live at x~0.25 m, z~0.85-1.1 m) ----
    HAND_X       = 0.25      # forward distance of both hands [m]
    HAND_Y       = 0.15      # lateral offset of each hand from the body mid-plane [m]
    DIP_Z        = 0.85      # height of the apple when held (start of the throw stroke) [m]
    CATCH_Z      = 0.90      # height at which the receiving palm meets the apple [m]
    V_TARGET     = np.array([-0.0, 0.70, 1.76])   # (vx, |vy| toward the other hand, vz) wanted at release
    STROKE       = 0.26      # length of the palm stroke [m]
    STROKE_TIME  = 0.26      # duration of the stroke [s]
    OPEN_AT      = 0.42      # fraction of the stroke at which the fingers begin to open
    PREROLL_TIME = 3.0       # silent warm-up so the standing robot reaches static equilibrium [s]
    SETTLE_TIME  = 0.60      # time the apple is simply held (visible) before the throw [s]
    CATCH_K      = 0.5       # palm moves at this fraction of the apple's speed at impact (cushioning)
    CLOSE_DELAY  = 0.12      # wait until the apple has settled on the palm before curling the fingers [s]
    CLOSE_TIME   = 0.40      # slow, gentle finger closure [s]
    LEVEL_TIME   = 0.20      # time to level the palm after the touch [s]
    CUSHION_DECEL= 6.0       # palm deceleration after the touch [m/s^2]
    DT_PLAN      = 0.010     # sample time of the planned joint trajectories [s]

    def __init__(self, model, data):
        self.m, self.d = model, data
        m = model
        j = lambda s, n: f"{s}_{n}_joint"
        self.arm_jid  = {s: [m.joint(j(s, n)).id for n in ARM_JOINTS] for s in SIDES}
        self.arm_qadr = {s: np.array([m.jnt_qposadr[i] for i in self.arm_jid[s]]) for s in SIDES}
        self.arm_dadr = {s: np.array([m.jnt_dofadr[i] for i in self.arm_jid[s]]) for s in SIDES}
        self.arm_act  = {s: np.array([m.actuator(j(s, n)).id for n in ARM_JOINTS]) for s in SIDES}
        self.hand_act = {s: np.array([m.actuator(j(s, n)).id for n in HAND_JOINTS]) for s in SIDES}
        self.hand_qadr= {s: np.array([m.jnt_qposadr[m.joint(j(s, n)).id] for n in HAND_JOINTS]) for s in SIDES}
        self.lo = {s: m.jnt_range[self.arm_jid[s]][:, 0] for s in SIDES}
        self.hi = {s: m.jnt_range[self.arm_jid[s]][:, 1] for s in SIDES}
        self.wrist = {s: m.body(f"{s}_wrist_yaw_link").id for s in SIDES}
        self.apple_gid = m.geom("apple_geom").id
        self.apple_qadr = m.jnt_qposadr[m.joint("apple_free").id]
        self.apple_dadr = m.jnt_dofadr[m.joint("apple_free").id]
        self.KP, self.KV = m.actuator_gainprm[:, 0], -m.actuator_biasprm[:, 2]   # position-actuator gains
        # all collision geoms that belong to a hand (palm lives on the wrist_yaw link, fingers on *_hand_* links)
        self.hand_geoms = {}
        for s in SIDES:
            bodies = {b for b in range(m.nbody)
                      if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b).startswith(f"{s}_hand")} | {self.wrist[s]}
            self.hand_geoms[s] = {g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies and m.geom_contype[g]}
        self.base_ctrl = m.key_ctrl[0].copy() if m.nkey else np.zeros(m.nu)    # legs/waist hold the 'stand' pose
        self._ikd = mujoco.MjData(m); self._snap = None
        self.thrower, self.catcher = "left", "right"
        self.reset()

    # ------------------------------------------------------------------ geometry / kinematics
    @staticmethod
    def R_hand(side, tilt):
        """Wrist-frame orientation: fingers forward, palm normal up, tilted by `tilt` about world x
        (positive tilt turns the palm toward the opposite hand)."""
        cols = [(1, 0, 0), (0, 0, -1), (0, 1, 0)] if side == "left" else [(1, 0, 0), (0, 0, 1), (0, -1, 0)]
        return Rx(SGN[side] * tilt) @ np.array(cols, float).T

    @staticmethod
    def h_loc(side):
        """Apple-centre point of the hand, expressed in the wrist_yaw_link frame (above the palm, between
        the thumb (back stop) and the finger pads)."""
        return np.array([0.085, -0.060 * SGN[side], 0.0])

    def sync_ik_model(self):
        """Copy the *live* configuration (floating base, waist, legs ...) into the private IK data, so that
        IK targets expressed in the world frame account for any sway/drift of the standing robot."""
        self._ikd.qpos[:] = self.d.qpos

    def ik(self, side, p_t, R_t, q0, iters=200, tol=(1e-5, 1e-4), lam=2e-2, posture=None):
        """Damped-least-squares IK for the 7 arm joints (uses a private MjData; the sim is untouched)."""
        m, dd, bid, dofs = self.m, self._ikd, self.wrist[side], self.arm_dadr[side]
        q = np.array(q0, float)
        for _ in range(iters):
            dd.qpos[self.arm_qadr[side]] = q
            mujoco.mj_kinematics(m, dd); mujoco.mj_comPos(m, dd)
            R = dd.xmat[bid].reshape(3, 3); p = dd.xpos[bid] + R @ self.h_loc(side)
            ep, er = p_t - p, so3_log(R_t @ R.T)
            if np.linalg.norm(ep) < tol[0] and np.linalg.norm(er) < tol[1]:
                break
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv)); mujoco.mj_jac(m, dd, jp, jr, p, bid)
            J = np.vstack([jp[:, dofs], jr[:, dofs]]); e = np.concatenate([ep, er])
            g = J.T @ e + (0.02 * (posture - q) if posture is not None else 0)
            dq = np.linalg.solve(J.T @ J + lam * np.eye(7), g)
            n = np.linalg.norm(dq)
            if n > 0.15: dq *= 0.15 / n
            q = np.clip(q + dq, self.lo[side], self.hi[side])
        return q, np.linalg.norm(ep), np.linalg.norm(er)

    def hand_pose(self, side):
        d = self.d; R = d.xmat[self.wrist[side]].reshape(3, 3)
        return d.xpos[self.wrist[side]] + R @ self.h_loc(side), R

    def apple_state(self):
        d = self.d
        return d.qpos[self.apple_qadr:self.apple_qadr + 3].copy(), d.qvel[self.apple_dadr:self.apple_dadr + 3].copy()

    def apple_touches(self, side):
        d = self.d
        for c in d.contact[:d.ncon]:
            if c.geom1 == self.apple_gid and c.geom2 in self.hand_geoms[side]: return True
            if c.geom2 == self.apple_gid and c.geom1 in self.hand_geoms[side]: return True
        return False

    # ------------------------------------------------------------------ planning
    def plan_hold_poses(self):
        """Nominal 'ready' joint poses (IK, upright base): thrower holds the apple low with the palm tilted
        toward the catcher; catcher waits with an open, tilted palm at the expected landing spot."""
        t, c = self.thrower, self.catcher
        sg, cg = SGN[t], SGN[c]
        self.tilt_throw = np.arctan2(self.V_TARGET[1], self.V_TARGET[2])
        v = np.array([self.V_TARGET[0], -sg * self.V_TARGET[1], self.V_TARGET[2]])
        self.u = v / np.linalg.norm(v)                                     # stroke / release direction
        q0 = np.zeros(7); q0[4] = -1.57 * sg
        self.q_hold_thrower, *_ = self.ik(t, np.array([self.HAND_X, sg * self.HAND_Y, self.DIP_Z]),
                                          self.R_hand(t, self.tilt_throw), q0, iters=500, tol=(1e-6, 1e-6))
        q0c = np.zeros(7); q0c[4] = -1.57 * cg
        self.q_ready_catcher, *_ = self.ik(c, np.array([self.HAND_X, cg * self.HAND_Y, self.CATCH_Z]),
                                           self.R_hand(c, self.tilt_throw), q0c, iters=500, tol=(1e-6, 1e-6))

    def fk_hand(self, side, q):
        dd = self._ikd; dd.qpos[self.arm_qadr[side]] = q; mujoco.mj_kinematics(self.m, dd)
        R = dd.xmat[self.wrist[side]].reshape(3, 3)
        return dd.xpos[self.wrist[side]] + R @ self.h_loc(side), R

    def plan_stroke(self):
        """Throwing stroke: the apple-centre point moves from the held pose along the release direction u
        with a minimum-jerk profile (palm orientation constant).  Solved with IK on the LIVE base pose."""
        self.sync_ik_model()
        t = self.thrower
        H0, R = self.fk_hand(t, self.q_hold_thrower)
        n = int(round(self.STROKE_TIME / self.DT_PLAN))
        Q = [self.q_hold_thrower]
        for k in range(1, n + 1):
            Q.append(self.ik(t, H0 + self.STROKE * minjerk(k / n) * self.u, R, Q[-1], iters=200, tol=(1e-6, 1e-6))[0])
        self.stroke_Q = np.array(Q)

    def plan_catch(self, t_now):
        """Called once, right after the apple has left the thrower's hand.  Predicts the ballistic landing
        from the *measured* apple state and plans the catcher's palm path to meet the apple."""
        c = self.catcher
        self.sync_ik_model()
        p, v = self.apple_state()
        zc = self.CATCH_Z
        disc = v[2] ** 2 + 2 * G * (p[2] - zc)
        tl = (v[2] + np.sqrt(max(disc, 0))) / G                       # time until apple centre is at height zc
        p_land = p + v * tl - 0.5 * np.array([0, 0, G]) * tl ** 2
        v_land = v - np.array([0, 0, G]) * tl
        self.pred = (t_now + tl, p_land.copy(), v_land.copy())
        vh = v_land / np.linalg.norm(v_land)                          # apple direction at impact
        speed = np.linalg.norm(v_land)
        tilt = np.arctan2(abs(v_land[1]), abs(v_land[2]))             # palm normal opposite to apple velocity
        R1 = self.R_hand(c, tilt)
        # --- Cartesian reference: quintic to (p_land, k*v_land) then constant-deceleration cushion
        H0, R0 = self.hand_pose(c)
        T1 = tl
        vh_hand = self.CATCH_K * v_land
        a_end = -self.CUSHION_DECEL * vh_hand / np.linalg.norm(vh_hand)
        coef = [quintic(H0[i], 0, 0, p_land[i], vh_hand[i], a_end[i], T1) for i in range(3)]
        tau_stop = np.linalg.norm(vh_hand) / self.CUSHION_DECEL
        n1 = int(np.ceil(T1 / self.DT_PLAN)); n2 = int(np.ceil(max(tau_stop, self.LEVEL_TIME) / self.DT_PLAN)) + 25
        R_lvl = self.R_hand(c, 0.0); w2 = so3_log(R1.T @ R_lvl)   # after touch: level the palm so the round apple cannot roll off
        w = so3_log(R0.T @ R1)
        Q = [self.q_ref_now(c)]
        for k in range(1, n1 + n2 + 1):
            t = k * self.DT_PLAN
            if t <= T1:
                P = np.array([np.polyval(co[::-1], t) for co in coef]); R = R0 @ so3_exp(minjerk(t / T1) * w)
            else:
                tt = min(t - T1, tau_stop)
                dirv = vh_hand / np.linalg.norm(vh_hand)
                P = p_land + dirv * (np.linalg.norm(vh_hand) * tt - 0.5 * self.CUSHION_DECEL * tt ** 2)
                R = R1 @ so3_exp(minjerk((t - T1) / self.LEVEL_TIME) * w2)
            Q.append(self.ik(c, P, R, Q[-1], iters=15, tol=(2e-4, 1e-3))[0])
        self.traj_catch = JointTraj(t_now, np.array(Q), self.DT_PLAN)
        self.t_land = t_now + tl

    def q_ref_now(self, side):
        return self.d.qpos[self.arm_qadr[side]].copy()

    # ------------------------------------------------------------------ control
    def arm_command(self, side, q, qd, qdd):
        """Position-actuator command with dynamics feed-forward:  tau = kp*(ctrl-q) - kv*qdot
        => ctrl = q_ref + (kv*qd_ref + tau_ff)/kp,  tau_ff = M qdd + bias + joint friction."""
        m, d, dofs = self.m, self.d, self.arm_dadr[side]
        vec = np.zeros(m.nv); vec[dofs] = qdd; res = np.zeros(m.nv)
        mujoco.mj_mulM(m, d, res, vec)                                 # res = M(q) @ vec
        tau = res[dofs] + d.qfrc_bias[dofs] + m.dof_frictionloss[dofs] * np.tanh(qd / 0.02)
        a = self.arm_act[side]
        return q + (self.KV[a] * qd + tau) / self.KP[a]

    def finger_cfg(self, side, cfg):
        return np.asarray(cfg) * SGN[side]

    # ------------------------------------------------------------------ episode management
    def reset(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        if m.nkey: mujoco.mj_resetDataKeyframe(m, d, 0)
        self.phase = "HOLD"
        self.traj_throw_obj = None; self.traj_catch = None
        self.t_throw0 = 1e9; self.t_open0 = 1e9            # (set after the pre-roll)
        self.t_release = None; self.t_catch_contact = None; self.t_close0 = None
        self.release_state = None; self.pred = None; self.contact_cnt = 0
        self.plan_hold_poses()
        t, c = self.thrower, self.catcher
        d.qpos[self.arm_qadr[t]] = self.q_hold_thrower
        d.qpos[self.arm_qadr[c]] = self.q_ready_catcher
        d.qpos[self.hand_qadr[t]] = self.finger_cfg(t, FING_CRADLE)
        d.qpos[self.hand_qadr[c]] = self.finger_cfg(c, FING_CATCH)
        mujoco.mj_forward(m, d)
        # initial condition: the apple starts inside the thrower's cradle (afterwards only physics moves it)
        p, _ = self.hand_pose(t)
        d.qpos[self.apple_qadr:self.apple_qadr + 3] = p
        d.qpos[self.apple_qadr + 3:self.apple_qadr + 7] = [1, 0, 0, 0]
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        # pre-roll: let the standing robot (+apple in its hand) settle into static equilibrium.
        # Done once; later resets just restore the stored settled state (instant, no freeze).
        if getattr(self, "_snap", None) is None:
            while d.time < self.PREROLL_TIME:
                self.step_control(); mujoco.mj_step(m, d)
            self.plan_stroke()                                  # plan on the settled, leaning robot
            self._snap = (d.time, d.qpos.copy(), d.qvel.copy(), d.act.copy() if m.na else None)
        else:
            tm, qp, qv, act = self._snap
            d.time = tm; d.qpos[:] = qp; d.qvel[:] = qv
            if act is not None: d.act[:] = act
            mujoco.mj_forward(m, d)
        self.t_throw0 = d.time + self.SETTLE_TIME
        self.t_open0 = self.t_throw0 + self.OPEN_AT * self.STROKE_TIME
        self.traj_throw_obj = JointTraj(self.t_throw0, self.stroke_Q, self.DT_PLAN)

    def step_control(self):
        """Call once per physics step, BEFORE mj_step."""
        m, d = self.m, self.d
        t, c = self.thrower, self.catcher
        ctrl = self.base_ctrl.copy()
        now = d.time
        z7 = np.zeros(7)
        # ---------------- thrower arm
        if now < self.t_throw0:
            q, qd, qdd = self.q_hold_thrower, z7, z7
        else:
            q, qd, qdd = self.traj_throw_obj.at(now)
        ctrl[self.arm_act[t]] = self.arm_command(t, q, qd, qdd)
        # ---------------- thrower fingers: cradle -> open
        a = np.clip((now - self.t_open0) / 0.06, 0, 1)
        ctrl[self.hand_act[t]] = self.finger_cfg(t, FING_CRADLE * (1 - a) + FING_OPEN * a)
        # ---------------- release detection (physics decides when the apple leaves the hand)
        if self.t_release is None and now > self.t_throw0 + 0.03:
            p, v = self.apple_state()
            if (not self.apple_touches(t)) and v[2] > 0.5: self.contact_cnt += 1
            else: self.contact_cnt = 0
            if self.contact_cnt >= 5:
                self.t_release = now; self.release_state = (p.copy(), v.copy()); self.phase = "FLIGHT"
                self.plan_catch(now)
        # ---------------- catcher arm
        if self.traj_catch is None:
            q, qd, qdd = self.q_ready_catcher, z7, z7
        else:
            q, qd, qdd = self.traj_catch.at(now)
            if now > self.traj_catch.t0 + self.traj_catch.T: q, qd, qdd = self.traj_catch.Q[-1], z7, z7
        ctrl[self.arm_act[c]] = self.arm_command(c, q, qd, qdd)
        # ---------------- catcher fingers: open until the apple touches the palm, then close around it
        if self.t_catch_contact is None and self.t_release is not None and self.apple_touches(c):
            self.t_catch_contact = now; self.t_close0 = now + self.CLOSE_DELAY; self.phase = "CATCH"
        a = 0.0 if self.t_close0 is None else np.clip((now - self.t_close0) / self.CLOSE_TIME, 0, 1)
        ctrl[self.hand_act[c]] = self.finger_cfg(c, FING_CATCH * (1 - a) + FING_CRADLE * a)
        d.ctrl[:] = ctrl

# ----------------------------------------------------------------------------------------------
# 3.  model loading / main
# ----------------------------------------------------------------------------------------------
def load_model(xml=None, g1_dir=DEFAULT_G1_DIR):
    if xml is None:
        xml = os.path.join(g1_dir, SCENE_NAME)
        if not os.path.exists(xml):
            if not os.path.exists(os.path.join(g1_dir, "scene_with_hands.xml")):
                sys.exit(f"Cannot find scene_with_hands.xml in {g1_dir}. Pass --g1-dir <path to menagerie/unitree_g1>.")
            with open(xml, "w") as f: f.write(SCENE_XML)
            print("wrote", xml)
    m = mujoco.MjModel.from_xml_path(xml)
    m.opt.timestep = 0.001                      # small step: robust fast contacts (apple vs fingers)
    return m

def run_headless(m, d, ctl, after=2.5):
    t_end = ctl.t_throw0 + after
    while d.time < t_end:
        ctl.step_control(); mujoco.mj_step(m, d)

def report(m, d, ctl):
    t0 = ctl.t_throw0
    if ctl.release_state is None:
        print("apple was never released"); return
    p, v = ctl.apple_state(); H, _ = ctl.hand_pose(ctl.catcher)
    print(f"release   t={ctl.t_release - t0:5.3f}s  pos={ctl.release_state[0].round(3)}  vel={ctl.release_state[1].round(2)} m/s")
    print(f"predicted landing t={ctl.pred[0] - t0:5.3f}s  pos={ctl.pred[1].round(3)}")
    if ctl.t_catch_contact is not None:
        print(f"first touch of the catching hand t={ctl.t_catch_contact - t0:5.3f}s")
    caught = np.linalg.norm(p - H) < 0.03 and np.linalg.norm(v) < 0.1
    print("final apple-to-hand offset [m]:", (p - H).round(3), "->", "CAUGHT" if caught else "MISSED")

def run_viewer(m, d, ctl, speed=0.5, loop=False, hold_after=2.5):
    """Frame-based pacing: every 1/60 s of wall time advance exactly speed/60 s of simulation, then sync.
    (Sleeping once per 1 ms physics step is jerky on Windows, whose sleep resolution is ~15 ms.)"""
    import mujoco.viewer
    FPS = 60.0
    steps_per_frame = max(1, int(round(speed / FPS / m.opt.timestep)))
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = [0.25, 0.0, 0.95]; v.cam.distance = 1.6; v.cam.azimuth = 160; v.cam.elevation = -10
        next_frame = time.perf_counter()
        while v.is_running():
            for _ in range(steps_per_frame):
                ctl.step_control(); mujoco.mj_step(m, d)
            v.sync()
            next_frame += 1.0 / FPS
            lag = next_frame - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            elif lag < -0.1:                     # a heavy frame (e.g. catch planning): don't fast-forward afterwards
                next_frame = time.perf_counter()
            if d.time > ctl.t_throw0 + hold_after:
                if not loop:
                    while v.is_running(): v.sync(); time.sleep(0.03)      # leave the final pose on screen
                    break
                ctl.reset(); next_frame = time.perf_counter()

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Unitree G1 tosses an apple from one bare hand to the other.")
    ap.add_argument("--xml", help="your own scene XML (must contain the 'apple' body, see SCENE_XML)")
    ap.add_argument("--g1-dir", default=DEFAULT_G1_DIR, help="path to mujoco_menagerie/unitree_g1")
    ap.add_argument("--headless", action="store_true", help="no viewer: simulate and print a report")
    ap.add_argument("--speed", type=float, default=0.5, help="viewer playback speed (1 = real time, 0.25 = slow motion)")
    ap.add_argument("--loop", action="store_true", help="repeat the demo until the viewer is closed")
    args = ap.parse_args()
    m = load_model(args.xml, args.g1_dir); d = mujoco.MjData(m)
    ctl = AppleToss(m, d)
    if args.headless:
        run_headless(m, d, ctl); report(m, d, ctl)
    else:
        run_viewer(m, d, ctl, args.speed, args.loop)
