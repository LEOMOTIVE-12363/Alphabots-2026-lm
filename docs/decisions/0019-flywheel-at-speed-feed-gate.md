# 0019 — Flywheel-at-speed feed gate (speed-only)

- Date: 2026-10-26
- Status: Accepted
- Supersedes: [0011 — Immediate feed with only flywheel-zero gating](0011-immediate-feed-only-flywheel-zero-gate.md)
  for the immediate-feed policy; restores the 0007-era gate with the Hood
  condition removed

## Context

0011 removed the 0007 readiness gate so a held shoot button feeds on the
command's first `execute`. Field use then exposed the opposite failure mode:
with a hopper full of balls, the first ball is pushed against a flywheel that is
still accelerating and jams there — exactly what the 0007 gate prevented. The
user confirmed the observation against the code and requested the gate back:
the shooter must reach its target speed before feeding starts.

The 0007 gate also required the Hood to be at angle, which had its own field
risk (an unreachable hood target never releases the balls). This decision
restores the gate on flywheel speed only.

## Decision

While the robot is enabled and the source controller has control priority, the
shoot commands first drive Drum and Hood to their targets; Indexer and Feeder
run only while the measured flywheel speed is within
`ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS` (±1.0 rot/s) of the
Flywheel-Speed-Limit-scaled target and that condition has held for
`ShootingConstants.READY_DEBOUNCE_SEC` (0.15 s). This applies to RB on either
port (`Shooting(false)`), A on port 0 (`Shooting(true)`, 50 A current feed), and
X on port 0 (`Manual`, gated against its own 50 rot/s target).

- Startup: a fresh press runs a SPIN_UP phase; feeding starts once the window
  is earned.
- Continuous gating: a measured mid-volley speed loss (or a target edit, whose
  old measurement reads below the new target) pauses the feed path on that very
  loop; Drum and Hood keep driving. Feeding auto-resumes without releasing the
  button once speed recovers and the window is re-earned. The retract timer
  freezes during these pauses.
- 0% limit: still stops feeding on its detection loop; a positive value resumes
  through the same spin-up-and-window path, not on that loop.
- Hood position never gates feeding (`hoodReady` stays a logged diagnostic);
  `REASON_HOOD_NOT_AT_ANGLE` is removed from the reason vocabulary.
- UNJAM still runs its back-off to completion regardless of readiness, then
  routes through PAUSED: post-unjam feeding re-earns a full stable window (the
  readiness debounce resets at jam entry) instead of resuming instantly.

## Retained behavior and interfaces

- All button bindings, controller priority, targets, feed speeds, gains, and
  the Shooting-versus-manual-reverse write conflict remain unchanged.
- The automatic jam threshold (90 A), confirmation window (0.2 s, feeding-time
  only), reverse outputs, back-off duration, and retract-timer freezing during
  0% pauses and UNJAM remain unchanged from 0011/0007.
- All NetworkTables and log key names are unchanged. `Robot Action` gains the
  `Spinning Up` display (the enum value already existed) while feeding waits,
  and `Feed Blocked Reason` shows `Flywheel not at speed`, `Stabilizing shot`
  (re-introduced from the 0007 era), `Flywheel limit is 0%`, or
  `Clearing indexer jam`. `OperatorStatus`'s no-command fallback still looks
  only at the 0% limit.
- `Shooting/drumReady`, `Shooting/hoodReady`, and `Shooting/stableReady` remain
  measurement-based log keys; `stableReady` now also mirrors the gate.

## Consequences

Feeding no longer starts on the first `execute`; the operator sees a short
spin-up delay proportional to real flywheel acceleration (the sim snaps almost
instantly, the real drum takes real time). Balls in the indexer wait for the
flywheel instead of jamming against it. Tight tolerance plus debounce may cause
brief pause/resume cycling under real voltage sag with a full hopper; the
tolerance and window stay at their pre-existing values pending real-robot feel,
and may be revisited (e.g. a percentage-of-target threshold) after on-robot
checks with the `Drum Target/Actual` and `Feed Allowed` panel widgets.

### Verification (local, not on-robot)

Full `./gradlew build` (spotless included): 32 suites / 323 tests / 0 failures
/ 0 errors / 0 skipped — same suite count as the 03a52d7 baseline, with the
immediate-feed pins rewritten to gate semantics (first-execute stop,
spin-up-then-feed, same-loop pause on sag/target edit, debounced auto-resume,
post-unjam re-gating, panel `Spinning Up`/`Stabilizing` phases, Manual gate,
speed-only hood pin). A 75-second `simulateJava` smoke started completely with
no exception and one pre-existing maple-sim loop overrun. Not verified on the
real robot: spin-up feel, tolerance behavior under battery sag, and current
spike shapes around resume remain on-robot checks.
