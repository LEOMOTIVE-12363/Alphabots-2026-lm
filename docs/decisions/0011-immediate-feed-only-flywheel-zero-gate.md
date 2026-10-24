# 0011 — Immediate feed with only flywheel-zero gating

- Date: 2026-10-24
- Status: Accepted
- Supersedes: [0007 — Continuous feed readiness gate](0007-continuous-feed-readiness-gate.md)
  for drum-at-speed, hood-at-angle, and stable-readiness feed gating

## Context

0007 made feeding wait for the drum and Hood to reach their targets, continuously
paused feeding when either measurement left tolerance, and required a stable
readiness window before startup or recovery. The demo requirement is now
immediate feeding from the existing shoot controls: drum and Hood still pursue
their targets, but their measured readiness no longer decides whether the feed
path runs.

This is a deliberate feeding-policy change, not a claimed vision fix. No
AprilTag or other vision root cause has been confirmed for the field behavior,
and this decision does not identify one. The shooting paths covered here must
remain independent of vision, AprilTags, pose, distance, and aiming data.

## Decision

While the robot is enabled and the source controller has control priority, a held
shoot command drives Indexer and Feeder at its existing feed targets on its first
`execute`. Drum and Hood are still driven to their original targets on the same
loop, and IntakeRoller keeps its existing shooting behavior. This applies to:

- RB on controller 1 or controller 0: `Shooting(false)`, retaining normal
  velocity-mode feeding;
- A on controller 0: `Shooting(true)`, retaining the 50 A current requests for
  Indexer and Feeder;
- X on controller 0: `Manual`, retaining its existing velocity-mode feed targets.

The only remaining shot-readiness feed permit is `Flywheel Speed Limit (%) > 0`.
At 0%, forward feeding and the feed path stop; restoring any positive value
resumes feeding on that loop. Drum not yet at speed or sagging, Hood not yet at
angle or drifting, mid-volley target changes, and readiness debounce no longer
pause, delay, or grant permission to feed.

Automatic jam clearing still runs its full back-off, including while the flywheel
limit is 0%. After the back-off, a still-active command immediately resumes the
interrupted feeding state whenever the limit is positive; it does not pass
through a drum/Hood readiness wait or a fresh stable window. If the limit remains
0%, forward feeding stays stopped only until the limit becomes positive.

## Retained behavior and interfaces

- Button release, controller disconnect, loss of control priority, command
  interruption, and robot disable still stop the corresponding command and its
  feed outputs. Controller 1 priority, controller 0 fallback, and every binding
  remain unchanged.
- All drum/Hood targets, feed speeds, gains, and intake retract positions remain
  unchanged. Normal velocity feed and A's 50 A current feed remain distinct. The
  documented Shooting-versus-manual-reverse write conflict is intentionally
  unchanged.
- The automatic jam threshold, confirmation window, reverse outputs, and back-off
  duration remain unchanged. Paused and UNJAM time still do not pre-fill the jam
  confirmation window. The retract timer remains frozen during a 0% pause and
  throughout UNJAM, then resumes its remaining count.
- Existing log and NetworkTables key names remain unchanged. `Robot Action`,
  `Feed Allowed`, and `Feed Blocked Reason` must report the active command's
  actual action and feed decision rather than a separate inferred gate.
- `Shooting/drumReady`, `Shooting/hoodReady`, and `Shooting/stableReady` remain
  measurement-based diagnostics. They must not be forced true to match the new
  feed decision, and may legitimately be false while feeding is allowed.

## Consequences

This decision explicitly replaces 0007's startup stable-readiness wait,
continuous raw-readiness pauses, pauses on speed/Hood/target changes, the
corresponding Manual gate, and post-UNJAM readiness re-gating. The original 0007
record remains as history. Its still-applicable work—same-loop 0% stops, retract
freezes, feeding-time-only jam confirmation, and command-reported panel
decisions—is retained rather than reopened by this change.

### Pre-integration local verification evidence

The original WIP was verified locally on `codex/demo-immediate-feed` (base
`dff89d4`) before integration with later `demo` changes. Behavior tests added or
updated under TDD covered all four entry points, first-execute feeding before
readiness with no vision/pose input, velocity versus 50 A current output,
sustained measurement faults and target changes, 0% stop and same-loop positive
recovery, release/disconnect/takeover/disable/interruption, re-press, automatic
jam behavior and retract-timer freezing, and dashboard consistency. That
pre-integration run passed `./gradlew spotlessApply` and the full
`./gradlew build` with 233 tests, 0 failures/errors/skipped; a roughly 200-second
`simulateJava` smoke started completely with no exception or error and only the
existing maple-sim loop-overrun warnings.

These are historical results for the original WIP only, not verification claims
for the integrated result on the later `demo` baseline. Neither these checks nor
this decision establishes real-mechanism ball flow, real current behavior,
operator feel, or real-robot deployment.
