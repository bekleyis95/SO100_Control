# SO100_Control — Post-Refactor Audit Report

**Last updated:** 2026-03-15 (3rd re-check)  
**Audited files:** `main.py`, `so100_robot_control/controllers/combined_controller.py`, `so100_robot_control/robot_interface.py`, `so100_robot_control/simulation/robot_kinematics.py`, `so100_robot_control/simulation/robot_simulation.py`, `so100_robot_control/teleop_devices/keyboard_listener.py`, `so100_robot_control/configs/configs.py`, `so100_robot_control/configs/robot_loader.py`, `robots/so100.example.yaml`, `pyproject.toml`

---

## Overall Fix Tracker

| # | Issue | Round Fixed |
|---|-------|-------------|
| 1 | Custom config destroyed when `mock=True` | ✅ Round 1 |
| 2 | Home position `> 0` vs `>= 0` edge case | ✅ Round 1 |
| 3 | Double-connect with disconnect in `_connect_and_home` | ✅ Round 1 |
| 4 | `lerobot` not in `pyproject.toml` dependencies | ❌ Still open |
| 5 | `_update_simulation` dead in keyboard+simulate mode | ✅ Round 1 |
| 6 | No hardware readback — pure dead-reckoning | ✅ Round 1 |
| 7 | `differential_ik` joint limits inverted / nonsensical | ✅ Round 2 |
| 8 | `calculateJacobian` wrong DOF count (all joints incl. fixed) | ✅ Round 2 |
| 9 | `grasp(close=True)` → 0.0 — inverted logic | ✅ Round 1 |
| 10 | `end_link` picked by fragile URDF array index `[-2]` | ❌ Still open |
| 11 | Magic 7cm / 7.3cm FK offsets not from URDF | ❌ Still open |
| N1 | `step_simulation` 10ms sleep blocked control loop | ✅ Round 2 |
| N2 | `stop()` called `p.disconnect()` with no physics server | ✅ Round 2 |
| N3 | `_resync_from_hardware` null-check was always True | ✅ Round 2 |
| 13 | `dataclass` dead import in `robot_loader.py` | ❌ Still open |
| 14 | `key.set_repeat` fires action-selection callbacks repeatedly | ❌ Still open |
| 15 | Joint 0 not keyboard-selectable | ❌ Still open |
| 16 | Backward-compat alias doesn't cover deleted module path | ❌ Still open |

---

## New Issues Found in Round 3

### N4. `robot_simulation.py:23` — Partial PyBullet physics server leak on URDF load failure

```python
def init_simulation(self):
    p.connect(p.GUI)            # ← physics server starts here
    ...
    self.robot = p.loadURDF(self.urdf_path, useFixedBase=True)  # ← can raise
```

`stop()` correctly guards the `p.disconnect()` call with `self.simulator.robot is not None`. But `self.simulator.robot` is only set after a **successful** `p.loadURDF()`. If `p.connect()` succeeds but `p.loadURDF()` raises (e.g., URDF file not found), the PyBullet GUI window and physics server are left running — `stop()` skips the disconnect because `self.simulator.robot is None`. The user is left with a dangling PyBullet process they can't close without killing the interpreter.

**Fix:** Catch the `loadURDF` exception in `init_simulation`, call `p.disconnect()` there, and re-raise.

---

### N5. `robot_simulation.py:150–154` — `differential_ik` return type is backward-incompatible

```python
# Before: returned np.ndarray of shape (num_joints,) — all URDF joints
# After:  returns np.ndarray of shape (n_act,)    — actuated joints only
return q
```

The internal `run()` modes are updated to scatter `q` back into a full array. But `differential_ik` is a public method — any external caller (notebook, script) that expected `num_joints`-sized output now silently gets an `n_act`-sized result and likely a shape mismatch downstream. The return type change is not documented in the signature or docstring.

**Fix:** Document the change prominently, or return a full `num_joints`-D array as before (with zeros for fixed joints).

---

### N6. `robot_simulation.py:154` — `differential_ik` resets fixed joints to 0° inside IK loop

```python
full = np.zeros(self.num_joints)          # zeros for ALL joints including fixed
for k, idx in enumerate(self._actuated_joints):
    full[idx] = q[k]
self.set_joint_states(np.degrees(full))   # set_joint_states calls resetJointState for all
```

`set_joint_states` calls `p.resetJointState(robot, i, rad)` for every index up to `min(num_joints, len(full))`. Fixed joints occupy slots that are zeroed out in `full`, so every IK iteration calls `resetJointState(robot, fixed_joint_idx, 0)`. PyBullet typically ignores this for `JOINT_FIXED` joints, but it is documented behaviour that fixed joints have a defined pose from URDF origin — resetting them to 0 is technically incorrect and creates unnecessary API calls on every IK iteration.

---

## Remaining Open Issues

### 4. `pyproject.toml` — `lerobot` not listed as a dependency

```toml
dependencies = [
    ...
    # lerobot motor driver (vendored copy included for offline use; a fresh
    # install can replace the vendor with:  pip install lerobot)
]
```

Imported unconditionally in `robot_interface.py:7` and `configs/configs.py:12`. A clean `pip install so100_control` without the vendored copy fails at import time. Must be a real dependency entry or a required extra (e.g. `[hardware]`).

---

### 10. `robot_kinematics.py:22` — `end_link` picked by fragile URDF array index

```python
self.end_link = self.robot.robot.joints[-2].child
```

Silently picks the wrong end-effector if URDF joint ordering changes. No named-link reference.

---

### 11. `robot_kinematics.py:88,101` — Two hardcoded offsets not derived from URDF

```python
T = self._trans(0, 0.07, 0.0)        # 7 cm base offset applied before chain

def _get_tcp_offset_transform(self):
    return sp.Matrix([..., [0, 1, 0, -0.073], ...])  # -7.3 cm TCP offset
```

Both are applied on top of what the URDF already encodes. If the URDF models these transforms, they are double-counted. Neither is cross-referenced to any URDF field or calibration source. FK and IK accuracy depend entirely on these magic numbers being correct.

---

### 13. `robot_loader.py` — `dataclass` dead import

```python
from so100_robot_control.configs.configs import (
    ...
    dataclass,    # only needed in configs.py, unused here
    ...
)
```

Triggers `flake8` F401. Remove it.

---

### 14. `keyboard_listener.py:19` — `key.set_repeat` unintentionally fires selection callbacks

```python
pygame.key.set_repeat(200, 50)
```

Holding keys `1`–`5` fires `_change_control_element` every 50ms, printing state noise and potentially toggling mode unintentionally. Arrow key movement is polled via `keys_pressed` each frame — repeat adds nothing for movement. The repeat should be removed or scoped to movement-only handling.

---

### 15. `keyboard_listener.py:26` — Joint 0 (shoulder pan) not selectable as Y-axis target

Keys `1`–`5` set `current_element` to 1–5. Element 0 is permanently wired to the X-axis in lower mode and cannot be redirected. The valid range in `_change_control_element` is `0–5`, but no key maps to 0.

---

### 16. `combined_controller.py:327` — Backward-compat alias doesn't help deleted module path

```python
CombinedController = RobotController   # inside controllers/combined_controller.py
```

The root-level `so100_robot_control/combined_controller.py` is deleted. Callers using `from so100_robot_control.combined_controller import CombinedController` get `ImportError`. The alias only works for callers who already use the new `controllers.` import path — i.e., callers that don't need the alias.

---

## Priority Fix Order (current)

1. **#4** — Add `lerobot` to `pyproject.toml` (breaks all clean installs)
2. **N4** — Fix `p.connect()` leak on URDF load failure (dangling process)
3. **N5** — Document or fix `differential_ik` return-type change (silent caller breakage)
4. **#11** — Validate/remove magic FK offsets against URDF content
5. **#10** — Replace `joints[-2].child` with a named end-effector link
6. **N6** — Filter fixed joints out of `set_joint_states` inside IK loop
7. **#13** — Remove dead `dataclass` import from `robot_loader.py`
8. **#14** — Remove or scope `pygame.key.set_repeat`
9. **#15** — Add keyboard binding for element 0
10. **#16** — Add a module-level shim at the old `combined_controller` path
