# ME206 - Assignment 4

**Group members:** Gurparas Singh (25110124), Veer Dhodia (25110103), George Mathew (25110118)

| Question | Folder | Description |
|---|---|---|
| Q1 | [`q1_g1_apple_toss/`](q1_g1_apple_toss/) | MuJoCo simulation of a Unitree G1 humanoid tossing an apple from one bare hand to the other |
| Q2 | [`q2_gravity_lab/`](q2_gravity_lab/) | Gravity Lab: interactive two-body gravitational simulator (HTML/JavaScript) |

- **Live Q2 simulator:** https://gurparassingh37.github.io/ME206-Assignment4/q2_gravity_lab/
- Q2 has its own detailed README: [`q2_gravity_lab/README.md`](q2_gravity_lab/README.md)

---

## Q1 - Unitree G1 apple toss (MuJoCo)

### Requirements
- Python 3.9+
- `pip install -r q1_g1_apple_toss/requirements.txt` (mujoco, numpy)
- MuJoCo Menagerie (G1 model with hands): https://github.com/google-deepmind/mujoco_menagerie

### Setup
1. Download or clone Menagerie.
2. Copy `q1_g1_apple_toss/scene_apple_toss.xml` into `mujoco_menagerie/unitree_g1/`
   (it includes the unmodified `scene_with_hands.xml` and only adds an apple body).

### Run
```
python q1_g1_apple_toss/apple_toss.py --g1-dir <path to mujoco_menagerie/unitree_g1>
python q1_g1_apple_toss/apple_toss.py --g1-dir <path> --loop --speed 1    # repeat in real time
python q1_g1_apple_toss/apple_toss.py --g1-dir <path> --headless          # no window, prints a report
```
On macOS use `mjpython` instead of `python`.

### Method
- The apple is a free rigid body (6-DoF free joint); only gravity and contacts move it.
- It is held in a bare-hand cradle (thumb as back stop, fingers curled) using the G1's existing Dex3 hand joints. No trays or attachments.
- Inverse kinematics (damped least squares using the Jacobian) gives the 7 arm joint angles along a minimum-jerk palm stroke.
- Release is not commanded: the apple leaves when the palm decelerates faster than gravity holds it.
- In flight, projectile motion predicts the landing point; IK then moves the other palm to meet the apple.
- The catching palm cushions the impact, levels out, and the fingers close.
- Joint feed-forward torque from the mass matrix improves tracking on the position actuators.

---

## Q2 - Gravity Lab (two-body simulator)

- Online: https://gurparassingh37.github.io/ME206-Assignment4/q2_gravity_lab/
- Locally: open `q2_gravity_lab/index.html` in any modern browser (no installation, no internet needed).
- Optional self-tests (Node.js 16+): `node q2_gravity_lab/tests/run_tests.js` (prints ALL TESTS PASSED, 10/10).
- Full description, physics and instructions: [`q2_gravity_lab/README.md`](q2_gravity_lab/README.md)
