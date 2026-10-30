# 0031 — Latched D-pad creep and Start heading reset

- Date: 2026-10-29
- Status: Accepted
- Amends: [0030 — Port-0 XYBA heading snap and D-pad creep](0030-port-zero-xyba-heading-dpad-creep.md), [0008 — Dual-controller routing with demo-port priority](0008-dual-controller-demo-priority.md)

## Context

The held port-0 D-pad crawl introduced by 0030 stops on release and cannot preserve a fine-positioning request while the driver uses the sticks or another Drive command owns the drivetrain. The intended interaction is a persistent toggle intent with explicit stop rules and safe lifecycle cleanup. The port-0 Start button also provides more value as the same heading reset already available on DS1 B than as a simulated-field reset.

This revision supersedes only the held-creep behavior and the Start/sim-reset assignment. The 0030 XYBA heading snaps, displaced-operation handling, exact command scaling, and all DS1 behavior remain unchanged.

## Decision

### DS0 D-pad toggle intent

Only DS port 0 uses the D-pad for latched, field-relative translation:

- With no latch active, a real press on any cardinal direction starts translation in that direction at a commanded magnitude of 10% of maximum linear speed.
- Releasing the D-pad does not stop the latch. With a latch active, a new press on either the same direction or the opposite direction on the same axis stops it. For example, right or left stops a right latch, while up/down leaves it unchanged. The same axis rule applies to an up/down latch; perpendicular presses are completely ignored.
- There is no timeout and no active-port-plus-held conditional edge masquerading as a fresh press. Raw POV edges are sampled only while the source pad is connected, in control, and the robot is enabled. A disconnected pad freezes its edge baseline, and the reconnect loop only resynchronizes that baseline.

### Joystick composition and Drive-command arbitration

One reusable latched drive command adds the D-pad offset to the original left-stick translation inputs (`-leftY × 0.9` and `-leftX × 0.9`). Right-stick X independently controls rotation. Neither stick cancels the intent, and centered sticks leave exactly the original 10% D-pad command.

The command reuses `DriveCommands.joystickDrive`, including the field-relative frame, red-alliance translation flip, 0029/#84 startup steer alignment and heading hold, and Dashboard Drive/Turn limits. The existing deadband inverse remains `d + (1-d) * sqrt(0.1)`, so the real WPILib range rescale and squaring produce a normalized D-pad magnitude of exactly 0.1 without changing the deadband, PID, or stick response.

Another Drive command may borrow the drivetrain without consuming the latch intent. The same latched command resumes once Drive is idle or owned only by its default command; the poll never blindly schedules over another Drive command. A latch requested during such a conflict is deferred rather than preempting the current command.

### Safety lifecycle

Disable, source-controller disconnect, and priority loss clear the intent and stop the crawl. Reconnecting or regaining control never restarts an old latch, including while POV remains held; a release and fresh physical press are required. A POV press made while disabled does not start on enable.

### DS0 Start and DS1

DS0 Start now calls the same `resetHeadingCommand` as DS1 B: blue resets to 0°, red resets to 180°, translation is preserved, and reset works while disabled. The former DS0 Start simulated-field reset has no physical button assignment. DS0 B remains the right-heading snap.

DS1 is unchanged: D-pad heading snaps, B heading reset, Back/Start LED selection, and every other current binding remain as before.

## Consequences

Fine translation persists after POV release and can coexist with both sticks, while explicit same-axis presses remain the only normal-operation stop inputs. Drive-command conflicts no longer swallow the intent, and lifecycle transitions cannot revive it unexpectedly. No NetworkTables key, log-key contract, Elastic layout, PID, deadband, or dependency changes are introduced.

Real-controller feel and real-robot motion remain unverified; the validation boundary is DriverStationSim, the real scheduler, and maple-sim physics.

## Verification

- Test-first implementation: the three affected suites produced 116 tests with 11 expected failures on the old behavior, then 116/116 passed after implementation.
- Independent review strengthened the joystick tests to assert measured field-relative components for forward, backward, and both strafe directions, including right-latch forward/backward composition and preservation of the right-creep component. Centered-stick exact 10% assertions were retained.
- Full `./gradlew test --offline`: 33 suites / 388 tests / 0 failures / 0 errors / 0 skipped.
