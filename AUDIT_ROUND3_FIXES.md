# Round 3 Audit — Verification & Fixes

**Date:** 2026-03-15

---

## Issues Claimed Open by Auditor — Confirmed Already Fixed

The round-3 audit report listed the following as "Still open". Direct file inspection showed all were already in place from previous rounds.

| # | Issue | Evidence |
|---|-------|----------|
| 10 | `end_link` picked by fragile URDF array index | `_END_LINK = "Fixed_Jaw"` constant defined; `self.end_link = _END_LINK` used |
| 13 | Dead `dataclass` import in `robot_loader.py` | Import not present in file |
| 14 | `key.set_repeat` fires selection callbacks | `pygame.key.set_repeat(0, 0)` in place |
| 15 | Joint 0 not keyboard-selectable | `pygame.K_0` mapped to `_change_control_element(0)` |
| 16 | Backward-compat shim missing | `so100_robot_control/combined_controller.py` shim file exists |
| 4  | `lerobot` not in package distribution | `lerobot*` in `[tool.setuptools.packages.find] include` |

---

## New Issues Fixed in Round 3

### N4 — PyBullet physics server leak on URDF load failure

**File:** `so100_robot_control/simulation/robot_simulation.py`

**Problem:** `p.connect(p.GUI)` succeeded but `p.loadURDF(...)` could raise (e.g. file not found). Because `self.robot` was only set after a successful load, the `stop()` guard (`self.simulator.robot is not None`) would skip `p.disconnect()`, leaving a dangling GUI window and physics process.

**Fix:** Wrapped `p.loadURDF` in try/except inside `init_simulation`; calls `p.disconnect()` before re-raising.

```python
try:
    self.robot = p.loadURDF(self.urdf_path, useFixedBase=True)
except Exception:
    p.disconnect()
    raise
```

---

### N5 — `differential_ik` return shape changed without documentation

**File:** `so100_robot_control/simulation/robot_simulation.py`

**Problem:** After the round-2 refactor, `differential_ik` returned an `(n_act,)` array (actuated joints only), silently breaking any caller that expected the original `(num_joints,)` shape.

**Fix:** Return a full `(num_joints,)` array with fixed joints preserved at their current values. Docstring updated with an explicit Returns section.

```python
# Return a full num_joints-D array so callers get a consistent shape.
full = np.array([self.joint_angles.get(i, 0.0) for i in range(self.num_joints)])
return full
```

Both `run()` modes that manually scattered the result were simplified to use the return value directly:

```python
# Before
ik_sol = self.differential_ik(...)
full = np.zeros(self.num_joints)
for k, idx in enumerate(self._actuated_joints):
    full[idx] = ik_sol[k]
self.set_joint_states(np.degrees(full))

# After
ik_sol = self.differential_ik(...)
self.set_joint_states(np.degrees(ik_sol))
```

---

### N6 — Fixed joints reset to 0° on every IK iteration

**File:** `so100_robot_control/simulation/robot_simulation.py`

**Problem:** The IK inner loop built a zeroed `full` array over all joints and passed it to `set_joint_states`, which called `p.resetJointState` for every joint including fixed ones. Fixed joints were forced to 0 on each iteration — unnecessary API calls and technically incorrect.

**Fix:** The IK loop now calls `p.resetJointState` only for actuated joint indices; fixed joints are never touched.

```python
# Before
full = np.zeros(self.num_joints)
for k, idx in enumerate(self._actuated_joints):
    full[idx] = q[k]
self.set_joint_states(np.degrees(full))

# After
for k, idx in enumerate(self._actuated_joints):
    p.resetJointState(self.robot, idx, q[k])
    self.joint_angles[idx] = q[k]
```
