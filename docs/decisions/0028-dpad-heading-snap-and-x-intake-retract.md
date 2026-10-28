# 0028 — D-pad heading snap and port-0 X intake retract

- Date: 2026-10-28
- Status: Accepted
- Amends: [0008 — Dual-controller routing with demo-port priority](0008-dual-controller-demo-priority.md)

## Context

The demo controls need a short, reliable way to point the intake/front of the robot at a driver-relative cardinal heading without holding a button throughout the turn. The previous D-pad assignments instead adjusted the flywheel percentage on port 0 and retracted the intake with D-pad left on either controller. Port-0 X also started the fixed-setpoint `Manual` shot, which conflicts with assigning X to intake retraction.

The heading feature must reuse the existing field-relative angle controller rather than introduce another controller or retune it. Releasing the D-pad must not abandon a partially completed turn, while explicit manual rotation and controller ownership loss must still return control safely.

## Decision

Both controllers use the four cardinal D-pad directions for heading snap under the existing connection-based priority rules. The blue-origin field targets and red-alliance equivalents are:

| D-pad | Blue target | Red target (blue + 180°) |
|---|---:|---:|
| Up | 0° | 180° |
| Down | 180° | 0° |
| Left | 90° | −90° |
| Right | −90° | 90° |

The alliance-adjusted target is captured when the press initializes the command. Red alliance therefore preserves the same driver-relative meaning for all four directions; left and right are not independently mirrored. The command rotates the chassis and never resets position or odometry.

`DriveCommands.joystickDriveToAngle(...)` is a goal-finishing variant of the existing `joystickDriveAtAngle(...)` path. Both use the same linear joystick handling, alliance-dependent translation, `ProfiledPIDController`, continuous input, and existing angle PID/profile constants. The original `joystickDriveAtAngle(...)` remains continuous; only the new variant appends `until(angleController::atGoal)`.

A D-pad tap continues after POV release until the angle controller reaches its goal, then ends and lets the default joystick drive resume. Moving right-stick X beyond the existing `DriveCommandsConstants.DEADBAND` cancels the snap immediately. Disconnecting the source controller or taking away its priority also cancels it through the existing per-source `onlyWhile(activePort == source)` wrapper.

Port-0 X now runs only `IntakeDeployZeroCommand(Direction.INWARD)`. Port-1 X remains unbound. The following old bindings are removed:

- port-0 X to `Manual`;
- port-0 D-pad up/down flywheel-percentage nudges;
- either controller's D-pad-left intake retract.

The `Manual` command implementation, tunables, gate behavior, and direct-command tests remain. It has no physical controller binding and can only be scheduled directly in code. Flywheel percentage remains controlled through the existing Elastic setting and its persistence path.

No angle PID/profile value, drive scaling, deadband, NetworkTables or log key, Elastic layout, dependency, or real-robot tuning parameter changes.

## Consequences

A short D-pad press no longer needs to be held while the chassis turns, and the operator can always override it with the right stick. The former D-pad flywheel nudges have no alternative physical binding. Intake retract has moved from D-pad-left to port-0 X, which has one unambiguous action: inward intake retraction and homing, with no Manual drum or feed request. The old D-pad bindings cannot accidentally compete with heading snap.

Finishing uses the existing controller's `atGoal()` semantics; no new heading tolerance was introduced. Desktop simulation verifies command and binding behavior, not real-robot steering dynamics or the feel of the reused gains.

## Verification

- Post-rebase focused integration run over `TeleopIntegrationTest`, `DriverControlsTest`, and `DriverControlsIntegrationAcceptanceTest`: 88 tests, 0 failures/errors/skips. Coverage includes all blue and red targets, release-then-converge, right-stick/disconnect/priority-loss cancellation, no D-pad flywheel nudges, D-pad-left heading rather than homing, and port-0-only X inward homing with no Manual output.
- Direct command coverage proves `joystickDriveToAngle` converges, finishes, and restores default drive, while `joystickDriveAtAngle` remains continuous.
- `./gradlew spotlessApply --offline` and `./gradlew build --offline --rerun-tasks`: BUILD SUCCESSFUL, including Spotless checks; 32 suites / 352 tests / 0 failures / 0 errors / 0 skipped.
- Bounded `./gradlew simulateJava --offline`: NT3/NT4 started without a port conflict and reached `Robot program startup complete`; no Exception/ERROR was logged. One 20 ms loop-overrun warning was recorded and is not attributed to the new bindings. Only this task's process group was terminated after startup.
- No robot deploy, physical-controller check, or real-robot heading/intake-homing validation is claimed.
