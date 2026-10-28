# 0027 — Require stall evidence (high current AND near-zero speed) to confirm UNJAM

- Date: 2026-10-28
- Status: Accepted
- Amends: [0024 — Release the speed gate at startup instead of cycling feed on every sag](0024-startup-only-feed-gate.md)
  (delivers the separate, measurement-driven jam-classification fix that 0024 explicitly
  deferred; every gate behavior from 0024 is unchanged)

## Context

0024 removed the speed-gate stutter path but stated that repeated `Clearing jam`
"requires a separate, measurement-driven jam-classification/back-off fix". The
remaining classifier was defective in a code-provable way: stator current above
90 A sustained for the 0.2 s debounce confirmed UNJAM **by current alone**, even
while the measured indexer speed showed the indexer was still turning. High
current while turning is not a stall — the indexer can draw hard while
accelerating a ball load — so the 0.5 s reverse would then reverse a healthy
feed, manufacturing a forward/reverse/stop cycle independent of any ball jam.

There is no real-robot wpilog in the repository, so how often this path fired
in the field is unproven. This decision fixes the classification defect itself;
it does not assert a measured field root cause.

## Decision

Confirm UNJAM only on combined stall evidence, feeding the **unchanged** 0.2 s
kRising debouncer one combined sample per loop:

- stator current above the unchanged `INDEXER_JAM_CURRENT_AMPS` (90 A), **and**
- measured `abs(velocity)` below the new `INDEXER_JAM_MAX_VELOCITY_ROTPS`
  (5.0 rot/s).

Under WPILib 2026.2.1 `Debouncer` kRising semantics, a sample equal to the
baseline resets the timer, so any single loop at healthy speed re-anchors the
window: the 0.2 s must be *continuously* stall-classified, and high current
while the indexer keeps turning can never trip UNJAM.

Everything else in the state machine is unchanged: the 90 A threshold, 0.2 s
confirmation, 0.5 s reverse at 50 A indexer / 30 A feeder current, UNJAM
revoking `feedGateReleased`, the post-unjam stable-window re-gating, force-feed
semantics (the latch bypasses only the post-unjam speed gate, never UNJAM
itself), clear samples fed during every non-feeding loop, and all state
transitions and outputs. No NetworkTables key, Elastic widget, motor gain,
target speed, feed speed, or controller binding changes. Energy-save (50 A
torque-current) feed still cannot trip jam detection, as before.

The 5.0 rot/s bound is a documented **preview value, not a calibrated one**:
6.25% of the existing 80 rot/s feed target, such that escaping it within the
0.2 s confirmation window requires 25 rot/s² — an acceleration healthy feeding
far exceeds — while a truly stalled ball cannot move at all.

## Consequences

High current with the indexer still turning can no longer trip UNJAM, so the
misclassification-driven forward/reverse/stop cycle is removed in software. A
true stall still reverses exactly as before.

Residual risks, exactly:

1. A real jam that keeps the indexer crawling above 5.0 rot/s is not classified
   as a stall and will not auto-clear; the operator falls back to 0%, button
   release, or manual reverse.
2. An unusually slow startup that stays below 5.0 rot/s while drawing more than
   90 A for 0.2 s could still clear spuriously.
3. The bound is not calibrated — no field log exists. Real current+velocity
   traces must validate or adjust it before hardware tuning is called final.

This change does not eliminate every mechanical jam or every possible stutter:
sustained severe flywheel speed loss and weak shots (0024's deliberate
trade-off) and electrical/mechanical faults remain out of scope. No field root
cause, real-robot smoothness, or final tuning is claimed.

## Verification

- TDD: RED on the old implementation failed exactly the two new behavior
  contracts (`highCurrentWhileIndexerIsTurningDoesNotUnjam` tripped at loop 10;
  `healthyIndexerSpeedResetsJamConfirmation` tripped at loop 0) while all 14
  pre-existing tests in the suite passed. GREEN after the classifier change;
  commits `b036f12` plus review-round fix `db29068` (the integration test
  originally commanded the sim motor to zero and now injects measurement-only
  stall evidence through a delegating `MotorIO` proxy).
- Delivery build (2026-10-28): `./gradlew spotlessApply` then
  `./gradlew build --offline --rerun-tasks` → BUILD SUCCESSFUL;
  **32 suites / 343 tests / 0 failures / 0 errors / 0 skipped**.
- Desktop-simulation smoke (same JDK): startup completes and the program runs
  disabled-steady with no Exception/ERROR from the robot program; the only loop
  overruns (4) trace to maple-sim `simulationPeriodic()` physics steps and one
  SmartDashboard update in the desktop JVM, not to robot code. The sim's
  near-instant speed response verifies program wiring and startup only; it is
  not evidence of real-robot current behavior or shot smoothness.
- Real-robot acceptance remains pending: after redeploy, retest 25% and 50%
  with the same ball count — healthy high current must not trip UNJAM, a true
  stall must still clear, and there must be no repeated stop/go. Until then
  only the software fix and local verification are claimed.
