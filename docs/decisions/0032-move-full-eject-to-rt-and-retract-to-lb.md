# 0032 — Move full eject to RT and intake retract to LB

- Date: 2026-10-30
- Status: Accepted
- Amends: [0008 — Dual-controller routing with demo-port priority](0008-dual-controller-demo-priority.md)

## Context

The operator wants the full-controller right trigger to own the complete outtake action previously on LB, and wants LB to retract and home the intake inward. The old RT feeder/indexer-only local reverse is removed rather than preserved as a second reverse mode. DS1 must remain unchanged.

## Decision

Only DS port 0 changes:

- Holding RT (>0.5) runs the complete reverse path: Indexer, Feeder, and IntakeRoller all receive −3 V. Releasing RT, disconnecting, losing priority, or disabling zeroes the requests. The old feeder/indexer-only local reverse has no binding or output path.
- LB now schedules `IntakeDeployZeroCommand(INWARD)`. It no longer runs reverse outputs, and disconnecting or losing priority still cancels the homing command.
- An RT outtake press clears the latched intake intent before reverse output is applied, matching the previous outtake lifecycle. DS1 LB retains that same complete outtake behavior and intake-latch cancellation.
- The reverse recompute remains requirement-free, so a concurrently running Shooting loop can overwrite it exactly as before. Press/release still recompute the active controller's truth, and release cleanup still runs while disabled. No replacement binding is created for the deleted local reverse.

DS1 keeps LB outtake and every other current binding. No PID, deadband, NetworkTables/log-key contract, Elastic layout, or dependency changes are introduced.

## Consequences

The full and demo controllers can hold different outtake inputs without a same-loop handoff clobbering the new owner's output: full RT is read only on DS0, while demo LB is read only on DS1. LB retract can coexist with the requirement-free reverse heartbeat but no longer contributes any reverse request itself. Real-controller feel and physical mechanism motion remain unverified.

## Verification

- Test-first mapping coverage failed before implementation: RT did not reverse the intake roller and LB did not perform the new inward-homing role.
- Real DriverStationSim/scheduler/maple-sim coverage proves full RT reverses all three mechanisms, full LB drives inward homing without reverse output, and demo LB still performs complete reverse.
- Reverse acceptance coverage verifies release cleanup, disabled behavior, full/demo handoff, and that releasing full LB does not clear an RT-held reverse.
- Focused suites: 116/116 passing. Full `./gradlew test --offline`: 33 suites / 389 tests / 0 failures / 0 errors / 0 skipped.
- No robot deploy or real-robot verification was performed.
