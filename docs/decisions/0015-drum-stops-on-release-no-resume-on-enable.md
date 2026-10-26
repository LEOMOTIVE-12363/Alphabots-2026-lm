# 0015 — Drum stops on shot release; re-enabling never spins it back up

- Date: 2026-10-26
- Status: Accepted

## Context

`Shooting.end()` inherited the upstream behavior of commanding the drum to a
"stow" velocity (`DrumConstants.StowVelocity = 47.5` rot/s, roughly 70–90 % of
the shot-speed range) when the shoot button was released. Two field-visible
consequences were reported on 2026-10-26:

1. After releasing the shoot button the drum kept *actively* spinning at the
   stow velocity — it looks like the shooter never stopped.
2. Phoenix 6 control requests persist on the motor controller until replaced.
   The drum has no default command and `Robot.disabledInit()` was empty, so
   the standing "spin at 47.5" request survived a disable. After re-enabling
   (even minutes later) the drum spun back up with no operator input.

The user accepted inertial coast-down but not the driven stow spin or the
re-enable spin-up.

## Decision

- `Shooting.end()` now calls `Drum.stop()` (0 V output, coast neutral mode)
  instead of writing the stow velocity. `Manual` already ended with `stop()`.
  On release the drum coasts out like every other shot mechanism; no standing
  velocity request remains, so re-enabling leaves it stopped.
- The now-unused `DrumConstants.StowVelocity` constant is removed.
- `Robot.disabledInit()` clears the drum (`Drum.stop()`) as defense in depth:
  whatever command path wrote last, no drum request survives a disable.
- Feeding behavior is unchanged ([0011](0011-immediate-feed-only-flywheel-zero-gate.md)
  immediate feed, the 0 % flywheel gate, and instant stop on release,
  disconnect, priority loss, interruption, or disable all stay as they are).

Trade-off accepted by the user: the flywheel no longer stays warm at stow
speed, so every new press re-accelerates the drum from rest — the first balls
of a fresh volley leave slower until the drum spins up. Predictability and
safety on the demo robot were judged more important than follow-up shot
latency.

## Verification

`TeleopIntegrationTest` assertions that pinned the stow setpoint after
command end now pin the stopped state (`ControlMode.DISABLED`), and a new
regression `reEnableAfterReleasedShotLeavesDrumStopped` replays the reported
scenario end to end: shoot → release → disable → long wait → re-enable, with
the drum asserted stopped and no feeding on every loop after re-enable, and a
fresh press still shooting normally afterwards. Simulation-level only; real
drum coast-down time and re-enable behavior on hardware remain to be verified
on the robot.
