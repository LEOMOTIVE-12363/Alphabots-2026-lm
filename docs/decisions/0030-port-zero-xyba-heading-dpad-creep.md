# 0030 — Port-0 XYBA heading snap and D-pad creep

- Date: 2026-10-29
- Status: Accepted
- Amends: [0028 — D-pad heading snap and port-0 X intake retract](0028-dpad-heading-snap-and-x-intake-retract.md), [0008 — Dual-controller routing with demo-port priority](0008-dual-controller-demo-priority.md)

## Context

The full controller needs cardinal heading snap on its letter buttons, freeing its D-pad for low-speed positioning. The simplified demo controller must keep its existing behavior. This changes only the current port-0 bindings; 0028 remains the historical record of the earlier shared D-pad snap and port-0 X retract mapping.

The letter buttons previously owned current-feed shooting, inward retraction, outward zeroing, and heading reset. Those operations must not move to replacement physical buttons, but their commands and direct-scheduling paths must remain available.

## Decision

### Heading snap

Only DS port 0 uses XYBA for goal-finishing heading snap through the existing `snapHeadingCommand` / `DriveCommands.joystickDriveToAngle` path. DS port 1 keeps the four D-pad heading snaps introduced by 0028.

| Direction | Blue target | Red target (blue + 180°) | DS0 input | DS1 input |
|---|---:|---:|---|---|
| Forward / intake forward | 0° | 180° | Y | D-pad up |
| Back / intake toward driver | 180° | 0° | A | D-pad down |
| Intake left | 90° | −90° | X | D-pad left |
| Front right | −90° | 90° | B | D-pad right |

A tap continues after button or POV release until the existing angle controller reaches its goal, then restores default joystick drive. Right-stick X beyond the existing deadband cancels the snap. Source-controller disconnect or priority loss also cancels it through the existing per-source ownership gate. Position and odometry are not reset.

### Port-0 D-pad creep

Only DS port 0 uses the D-pad for held, field-relative translation in the driver-perspective directions: up/down moves along the driver-relative X axis and left/right along the driver-relative Y axis. Red alliance retains the existing 180° translation flip. The commanded linear magnitude is 10% of `Drive.getMaxLinearSpeedMetersPerSec()` before the normal Dashboard Drive limit is applied; the default 100% limit therefore produces a command of exactly `0.1 × maxLinearSpeed`.

Each direction runs `DriveCommands.joystickDrive` while held, with no timeout. Releasing the POV immediately restores default joystick driving. Disconnect or priority loss stops the creep through the same per-source cancellation semantics as other held commands. This path reuses the existing field frame, red-alliance handling, 0029/#84 startup steer alignment, straight-line heading hold, and Dashboard Drive/Turn limits rather than duplicating or bypassing them.

`joystickDrive` applies `MathUtil.applyDeadband` and then squares the resulting magnitude. A raw `Math.sqrt(0.1)` input would therefore produce less than 0.1 because `applyDeadband` also rescales the range above the deadband. For deadband `d`, the D-pad input magnitude is:

```
d + (1 - d) * sqrt(0.1)
```

The real WPILib deadband transform maps this back to `sqrt(0.1)`, whose square is exactly 0.1 on the command side. This inverts the existing range rescale without changing the deadband, default joystick response, PID, or drive limits.

### DS1 and displaced operations

DS1 behavior is unchanged: D-pad still snaps in four directions, B still resets heading, Back/Start still control LED selection, and XYBA does not gain the full-controller operations.

The displaced port-0 operations receive no new physical assignments:

- A current-feed `Shooting(true)`;
- X inward intake retraction/homing;
- Y hood and outward intake zeroing;
- B heading reset.

Their commands and direct-scheduling coverage remain. DS1 B retains heading reset. No NetworkTables key, log-key contract, Elastic layout, PID, deadband, or dependency changes are introduced.

## Consequences

Port 0 now separates heading selection (XYBA) from fine translation (D-pad), while port 1 keeps the demonstrated control scheme. Short snap presses still finish after release; creep is strictly momentary and cannot outlive release or controller ownership.

The former port-0 mechanism operations cannot start accidentally from the new XYBA snaps. Tests schedule those commands directly to preserve their behavior and lifecycle coverage without inventing replacement buttons.

Desktop tests distinguish exact command scaling from maple-sim's measured steady-state speed droop. Real-controller feel and real-robot motion remain unverified.

## Verification

- Focused post-review run over `TeleopIntegrationTest`, `DriverControlsTest`, and `DriverControlsIntegrationAcceptanceTest`: 106 tests, 0 failures/errors/skips.
- Full `./gradlew test --offline`: 33 suites / 379 tests / 0 failures / 0 errors / 0 skipped.
- An executable assertion uses the real WPILib `MathUtil.applyDeadband` and verifies the squared normalized creep output is `0.1 ± 1e-12`; blue/red XYBA targets, release-to-finish, four-direction motion, six-second held creep, release/disconnect/priority-loss cleanup, and DS1 regressions are covered separately.
- The current-feed takeover acceptance test directly schedules `Shooting(true)` with the port-0 ownership gate, proves the 50 A current-feed state exists before takeover, then retains all original reverse-voltage assertions.
- Final `./gradlew spotlessApply --offline` and `./gradlew build --offline --rerun-tasks`: BUILD SUCCESSFUL, including Spotless; 33 suites / 379 tests / 0 failures / 0 errors / 0 skipped.
- Final bounded `./gradlew simulateJava --offline`: startup completed in about 6.52 seconds with NT3/NT4 and Phoenix started normally, no Exception/Error at/ERROR, and one 20 ms loop-overrun warning. Only this task's process group was terminated; no other process or robot deploy was touched.
