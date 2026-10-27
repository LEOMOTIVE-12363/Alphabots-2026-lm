# 0023 — Latched-intake intent survives mechanism conflicts

- Date: 2026-10-27
- Status: Accepted
- Amends: [0016 — Latched intake toggle](0016-latched-intake-toggle.md) (latch state is now an
  intent, not the command's schedule state) and [0022](0022-acceptance-fixes-accumulation-persistence-lt-edges.md)
  (the real-edge detection carries over unchanged)

## Context

Field report (2026-10-27): with the intake latched on, pressing shoot stopped
collection and it never came back — the operator had to press LT again. Under
0016 the latch state was exactly "the command is scheduled", so any
requirements conflict (Shooting/Manual interrupting IntakeCommand) silently
consumed the latch. The operator's rule is strict: LT is press-on / press-off,
and nothing except an outtake (plus the existing safety teardowns) may end
collection.

## Decision

The latch becomes an **intent** (`intakeLatchDesired`), independent of the
command's schedule state:

- A fresh LT press with the intent off schedules the intake IMMEDIATELY,
  preempting whatever mechanism command currently borrows the subsystems — a
  new press means "intake NOW" (this preserves the pre-existing
  LT-interrupts-Shooting behavior).
- While the intent is on and the intake command is not running, the poll
  reschedules it every loop **only when every required subsystem is free**
  (`CommandScheduler.requiring` check — scheduling blindly would interrupt the
  conflicting command, WPILib default). So Shooting/Manual/the homing retract
  merely borrow the subsystems; when they finish, collection resumes on its
  own.
- The intent clears on exactly: a second LT press, an LB outtake press, robot
  disable, and the source pad disconnecting or losing priority — those never
  resume on their own; press LT again (unchanged from 0016/0022).
- The 0022 real-edge detection is unchanged: raw per-port edges, disconnect
  freezes the edge state, reconnect only resynchronizes the baseline — control
  returning or a replug while LT is held is not a new press. The 0022
  awaiting-release machinery is subsumed: the intent is cleared on
  disable/disconnect/priority loss directly, so there is nothing to gate.
- All other bindings are untouched.

## Retained interfaces

No NetworkTables keys, log keys, panel strings, or layout changes. LED design
untouched.

## Verification (local; RED first, then green)

- RED on the unmodified tree: `shootingConflictDoesNotCancelTheIntakeLatchIntent`
  (latch on → RB press → intake yields → RB release → intake must resume by
  itself → second LT press stops it) and the Manual (X) variant both failed —
  the 0016 field report reproduced.
- GREEN: 32 suites / **334 tests / 0 failures / 0 errors / 0 skipped**
  (332 baseline + 2 new; every existing LT test from 0016/0022 passes
  unchanged, including LT-interrupts-Shooting, disable/disconnect/takeover
  teardown, and the takeover/reconnect no-restart cases). 60 s `simulateJava`
  smoke: startup complete, no exceptions.
- Not verified on the real robot: physical-pad LT edges, the preemption feel
  when pressing LT mid-shot, and the resume delay after a shot (one loop once
  the subsystems free up).

## Consequences

Collection now continues across shots the way the operator described. The
intake deploy stays extended between a finished shot and the automatic resume
(one-loop gap). Pressing LT while a shot runs still preempts the shot
(intentional, preserves operator expectation); if that should ever become
"wait instead of preempt", it is a one-line change in the toggle.
