# 0025 — Hood target survives Shooting.end

- Date: 2026-10-27
- Status: Accepted
- Related: [0015 — Drum stops on release](0015-drum-stops-on-release-no-resume-on-enable.md)
  (the drum half of the release/disable teardown; unchanged here),
  [0021](0021-widen-tolerance-force-feed-timeout.md) /
  [0022](0022-acceptance-fixes-accumulation-persistence-lt-edges.md) (the feed gate itself is
  untouched by this record), [0024](0024-startup-only-feed-gate.md) (this branch was rebased
  on top of 0024's gate restructure, which does not touch `end()`)

## Context

Field behavior (2026-10-27): after a volley ended — button release, or robot disable
mid-volley — the hood dropped back toward 0°, so the operator's Elastic-set angle never
survived a shot. The root cause was a hard-coded write in `Shooting.end()`:
`Hood.mInstance.setPositionRot(0)`. It was **not** an Elastic persistence problem: the input,
its persisted setting, and the dashboard target value were all intact; the exit path itself
was overwriting the mechanism request to 0° on top of them.

## Decision

Delete exactly that one write (one line removed from
`src/main/java/frc/robot/commands/ShootingCommand/Shooting.java`; nothing else in the file
changed). `Shooting` no longer writes a 0-degree hood request on exit: on button release,
controller disconnect, loss of priority, interruption, or disable, the last written request
simply stays, and while the robot is enabled the existing idle follower keeps applying the
**latest Elastic target** (`Hood Angle (%)`, 100% = 45° = 0.125 rot) on its next loops —
including after a disable/re-enable cycle, once the default command re-registers. The rest of
the teardown is unchanged: drum stop-and-coast (0015), feed-path stops, intake deploy position.
Manual (X) still writes its own independent `Manual/HoodDegree` tunable target, and the Y-key
homing/calibration path is untouched.

## Retained interfaces

No NetworkTables keys, log keys, panel strings, layout entries, Elastic settings, or
persistence changes. No gain, limit, or percent→angle mapping changes.

## Verification (local; RED first, then green)

- Two real-binding regressions added to `TeleopIntegrationTest` (real DS1 RB binding, real NT
  dashboard writes, real `CommandScheduler`):
  `shootingReleaseKeepsDashboardHoodTargetWithoutAutoZero` — release RB and assert on that
  very loop that the hood setpoint is still 0.125 rot and `/SmartDashboard/Hood Angle (%)`
  still 100; `disableDuringShootingKeepsDashboardHoodTargetWithoutAutoZero` — the same checks
  on the disable loop, four further disabled loops, and 25 loops after re-enabling without
  shooting.
- RED on the old code: both failed at exactly the same-loop setpoint assertion
  (`expected: <0.125> but was: <0.0>`) — the discriminating checks are immediate setpoint
  reads, not follower-restored values. GREEN after the one-line deletion.
- Full `./gradlew build --offline --rerun-tasks`: BUILD SUCCESSFUL, 32 suites /
  **336 tests / 0 failures / 0 errors / 0 skipped** (334 baseline + 2 new), spotless green.
  (Captured on the pre-rebase base `dae52ef`.)
- ~60 s `simulateJava --offline` smoke from this worktree: startup complete, no Exception /
  "Error at" / ERROR lines; the only warnings were the pre-existing maple-sim 20 ms loop
  overruns (see the task evidence log). (Also captured on the pre-rebase base.)

**Rebase onto 0024 (`origin/demo` `0a8d92e`) and re-verification on the combined tree.**
PR #74 restructured the same `Shooting`/`Manual` gate code and both READMEs' shooting
paragraphs, but left `end()` untouched, so the one-line deletion and both regressions
reapplied cleanly; the README sentence was re-appended to the **new** upstream paragraph text
in both languages (the old gating paragraph was not resurrected). Re-verified:

- The two named regressions PASS (`./gradlew test --offline --tests …
  TeleopIntegrationTest.shootingReleaseKeepsDashboardHoodTargetWithoutAutoZero --tests …
  TeleopIntegrationTest.disableDuringShootingKeepsDashboardHoodTargetWithoutAutoZero`).
- `./gradlew spotlessApply --offline` (no changes) and full `./gradlew build --offline
  --rerun-tasks`: BUILD SUCCESSFUL, 32 suites / **336 tests / 0 failures / 0 errors /
  0 skipped**.
- ~60 s `simulateJava --offline` on the combined tree: startup complete, no Exception /
  "Error at" / ERROR lines; two 20 ms loop-overrun warnings — one alongside Phoenix library
  initialization, one with a tracer of `robotPeriodic(): 0.043 s` (while
  `simulationPeriodic(): 0.0013 s`) — startup transients, not attributed to this change
  (full log in the task evidence).

## Consequences

The hood now holds the last dashboard angle between volleys instead of dropping toward 0°
after every shot; the operator no longer has to re-enter the Elastic angle after each volley.
Honest boundary on what "keeps the target" means: the **request** persists, but the measured
`Hood Actual Angle (%)` may legitimately read 0 — the relative encoder is zeroed at robot-code
startup, motor output is disabled while the robot is disabled, and physical motion or an
encoder reset cannot be hidden or faked from software. Not verified on the real robot: the
operator's feel of a hood that stays raised after shots, and the mechanism's long-hold
behavior (the holding request already existed whenever the enabled follower ran; brake-mode
semantics and current limits are unchanged).
