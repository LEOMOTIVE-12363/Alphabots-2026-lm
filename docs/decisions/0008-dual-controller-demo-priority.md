# 0008 — Dual-controller routing with demo-port priority

- Date: 2026-10-22
- Status: Accepted

## Context

Demo sessions need a simplified controller that a guest can hold without
triggering calibration or tuning bindings, while the full control scheme must
remain available as a fallback. Priority must be unambiguous and must not
depend on stick movement ("whoever moved last wins" is not acceptable for a
demo handoff). Until now the robot ran a single `CommandXboxController` on DS
port 1 carrying the full binding set.

## Decision

`util/DriverControls` owns two `XboxController` instances (ports 0 and 1) and
routes exclusively by connection state: port 1 (simplified demo controller)
wins whenever it is connected, port 0 (full controls) is used only while port
1 is absent, and with neither connected the drive axes read zero. The
`Action` enum maps each physical input once and marks actions as shared or
full-only:

- Shared (both ports): left-stick drive / right-stick-X turn, LT intake, LB
  full-chain reverse eject, RB `Shooting(false)`, D-pad left inward intake
  homing, B reset heading (blue 0° / red 180°, position kept).
- Full-only (port 0): A `Shooting(true)` current-feed, X `Manual`, Y hood
  zero + outward intake zero, RT feeder/indexer-only reverse, D-pad up/down
  flywheel percent, Start sim-only field reset.

Every binding creates one command instance per source port, gated by
`activePort() == source`; press/hold commands are wrapped in `onlyWhile`, so a
held command — including one-shot homing runs — is interrupted the moment its
controller disconnects or loses priority (both homing commands already stop
their motors in `end(interrupted)`).

The LB/RT reverse keeps its original momentary semantics: press writes the
reverse voltages once, a release binding zeroes them (fires on release,
disconnect, or priority loss), and a requirement-free heartbeat feeds the LED
ball-flow layer and operator panel. No subsystem ownership is taken, so a
concurrently running Shooting loop overwrites the reverse writes — that
documented conflict is intentionally preserved (2026-10-22 user decision;
automatic UNJAM covers real clearing needs).

The operator panel is published once, from `Robot`'s 10 Hz block (the
duplicate call in `RobotContainer.updateDashboardOutputs` from parallel
development was removed). New read-only keys: `Controller 0 Connected`,
`Active Controller` (-1 = neither) under `/SmartDashboard`, plus the
`Controls/ActiveController` log key; widgets added to the Elastic layout,
which moves to a dated file name (`elastic-layout-1022.json`) from this
change onward.

## Consequences

Two controllers can stay plugged in during demos: the guest controller (port
1) always wins, and unplugging it hands control to port 0 without restarting
anything. Drive scaling (left axes ×0.9), deadbands, thresholds, and every
command class are unchanged from the single-controller scheme. Verified by
`DriverControlsTest` (10 tests: connection priority, axis routing, per-port
firing, full-only isolation on port 1, release/disconnect/priority-loss
cancellation, disable cancellation, one-shot homing cancellation) and the
full `./gradlew build` + `simulateJava` smoke. Not verified on the real
robot: real DS port assignment, handoff feel, and eject effect on hardware.

## Revision 2026-10-23 — reverse recompute + disabled-safe cleanup

Codex's acceptance of dcdfb1e (real RobotContainer + CommandScheduler +
DriverStationSim + simulated mechanisms) reproduced two defects in the
original LB/RT wiring:

1. **Cleanup lost while disabled.** `onRelease` scheduled a plain `runOnce`
   via `trigger.onFalse`; the 2026 scheduler silently refuses commands that
   don't `runWhenDisabled`, so a release/disconnect during disable consumed
   the falling edge without running the zeroing — after re-enable the
   simulated Feeder still held −3 V.
2. **Handoff clobbering.** Press and release were separate scheduled
   commands; on a same-loop source switch the new owner's press ran first
   and the old owner's release then zeroed the request, so the active pad's
   held LB stopped feeding until re-pressed (both directions reproduced).

Fix (keeps the deliberately preserved Shooting-vs-reverse conflict and all
momentary semantics): reverse outputs are now a pure function of held state.
`DriverControls.isHeldByActiveController(Action)` reports whether the
controlling pad holds LB (full) / RT (local); `RobotContainer`'s press and
release bindings both call `applyReverseFeedDesired()`, which writes
Indexer/Feeder −3 V when either is held (IntakeRoller −3 V only on full),
zero otherwise, and forces all-clear while disabled. The press command is a
plain `runOnce` (never starts while disabled); the release command adds
`ignoringDisable(true)` (cleanup always lands). Both callbacks compute the
same truth, so execution order no longer matters, and overlapping LB/RT
releases no longer zero each other's channels. `MotorSubsystem` gained
`getVoltageSetpoint()` for request-level assertions.

Verification: 12 new acceptance tests (4 scheduler-level
`DriverControlsAcceptanceTest`, 8 integration
`DriverControlsIntegrationAcceptanceTest` driving the real container and the
simulated Feeder/Indexer/IntakeRoller) — both handoff directions keep −3 V,
disabled release/disconnect clear the request, pressing while disabled does
not start a reverse, normal release and enabled disconnect still stop, and
LB/RT overlap survives both release orders. Negative control (fix stashed):
the 6 behavioral tests fail on the old wiring; the 2 semantic pins pass on
both. Full `./gradlew build`: 125 tests, 0 failures; simulateJava smoke
clean (loop overruns trace to maple-sim physics, as before). Not verified
on the real robot: actual motor output behavior while disabled, handoff
feel, and disabled-period key semantics.

## Revision 2026-10-24 — recompute must run in execute(), not initialize()

Codex's recheck of f553950 (8 cases, 6 passed) reproduced one remaining
handoff defect on two paths: port 0 holds X (Manual) or A
(`Shooting(true)`), port 1 connects already holding LB → the feeder reads
0 V until LB is re-pressed. Cause: the reverse recompute ran in
`InstantCommand.initialize()`, which fires synchronously at schedule time
during the button-loop poll; the outgoing shooting command is cancelled
later, in the command-execution phase (`onlyWhile` → `isFinished`), and its
`end()` zeroes the feeder/indexer/roller after the new owner's write.

Fix: the reverse one-shots are now `FunctionalCommand`s whose `execute()`
performs the same `applyReverseFeedDesired()` recompute. Commands scheduled
this loop iterate after long-running ones, so the write lands after the
outgoing command's `end()` in the same scheduler loop. No binding-order
shuffle, no requirements (the rejected runEnd approach stays rejected), and
the deliberately preserved same-pad Shooting-overwrites-reverse conflict is
unchanged: the press still writes once and the next Shooting loop
overwrites it.

Tests: `DriverControlsIntegrationAcceptanceTest` gained
`acceptanceTakeoverFromManualKeepsNewOwnersReverse` and
`acceptanceTakeoverFromCurrentFeedKeepsNewOwnersReverse` (ported from
Codex's failing pair, extended to assert indexer and intake-roller requests
alongside the feeder at handover, after one held second, and after a
re-press). Negative control (fix stashed): both fail on the old code, the
8 pre-existing integration tests still pass. The class also gained an
`@AfterAll` that unregisters and nulls `Drive.mInstance`, the hygiene
ShootingTest's null-assertion relies on (pre-existing cross-class ordering
sensitivity). Full-tree `./gradlew build`: 176 tests, 0 failures (includes
the parallel LED session's uncommitted WIP tests; committed tree carries
155 + these 2). simulateJava smoke: clean startup; loop-overrun warnings
trace to maple-sim physics as before. Not verified on the real robot.

## Revision 2026-10-24 — reverse heartbeat no longer feeds LEDs

[0012](0012-dashboard-selected-led-only.md) supersedes the original statement
that the requirement-free reverse heartbeat feeds an LED ball-flow layer. The
heartbeat and momentary reverse semantics are unchanged, and the heartbeat
continues to feed `OperatorStatus` and the operator panel; it no longer changes
LED output. `LED Effective State` and `LED Flow Direction` are read-only
mechanism diagnostics, not the actual light effect or direction. Controller
routing, bindings, reverse behavior, commands, and `OperatorStatus` are
unchanged by this revision.

## Revision 2026-10-28 — D-pad headings and X retract

The current D-pad and X bindings are revised by
[0028](0028-dpad-heading-snap-and-x-intake-retract.md): all four cardinal
D-pad directions start heading snaps on both controllers, the old up/down
flywheel nudges and D-pad-left retract are removed, and port-0 X now runs
inward intake homing instead of `Manual`. The `Manual` command remains
implemented but has no controller binding. The binding lists above are
retained as the historical record of the original dual-controller routing.

## Revision 2026-10-29 — port-0 XYBA and D-pad creep

[0030](0030-port-zero-xyba-heading-dpad-creep.md) supersedes only the current port-0 button assignments: XYBA performs heading snap and the D-pad performs held 10% translation. DS1 behavior is unchanged. Displaced port-0 operations receive no new physical bindings, while their commands and direct-scheduling paths remain available.

## Revision 2026-10-29 — latched D-pad creep and Start heading reset

[0031](0031-latched-dpad-creep-and-start-heading-reset.md) supersedes the held port-0 D-pad creep introduced by 0030 and the full-only Start simulated-field reset listed above. All other controller routing and binding decisions remain unchanged.

## Revision 2026-10-30 — full eject on RT and intake retract on LB

[0032](0032-move-full-eject-to-rt-and-retract-to-lb.md) moves the full controller's complete outtake to RT and inward intake homing to LB, deleting the old RT local reverse. DS1 LB outtake and all other DS1 behavior remain unchanged.
