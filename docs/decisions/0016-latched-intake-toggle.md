# 0016 — Latched intake toggle (LT press on / press off)

- Date: 2026-10-26
- Status: Accepted

## Context

Since the dual-controller rework ([0008](0008-dual-controller-demo-priority.md)),
LT held-to-run collection: releasing the trigger stopped the `IntakeCommand`.
For demo driving the operator wants collection to continue without holding the
trigger, with the same one-key interaction on both controller ports. The eject
path (LB) stays momentary, but an outtake press must take over the feed path:
while collection is latched on, pressing LB has to stop collection and start the
reverse instead of fighting it. Before this change the two bindings could not
even coexist meaningfully — the running `IntakeCommand` re-wrote the roller,
indexer, and feeder every loop and simply overwrote the momentary -3 V reverse
writes.

## Decision

- LT is now a latch on both ports: the first press schedules one shared
  `IntakeCommand` instance, a second press cancels it. The latch state is
  exactly "the latched command is scheduled" — there is no separate boolean.
- The latch is torn down, and does not resume by itself, when:
  - the robot is disabled (the scheduler cancels it; re-enable stays idle),
  - the owning port disconnects or loses control priority (`onlyWhile` guard on
    `activePort == source`; a takeover or reconnect never re-fires it), or
  - any command with overlapping requirements is scheduled (Shooting, Manual,
    D-pad-left retract/homing), which interrupts it as before.
  In every case the operator presses LT again to restart collection.
- Pressing LB (outtake, either port) first cancels the latch, then the existing
  momentary reverse recompute writes the intake roller, indexer, and feeder to
  -3 V. Releasing LB zeroes them; collection does not resume afterwards.
- The cancel runs in a separate poll-phase `runOnce` bound to the EJECT press,
  deliberately NOT inside `applyReverseFeedDesired`: WPILib 2026 defers a
  cancel issued during the execute phase to loop end (`m_toCancelCommands`),
  so the latch's `end()` zero-writes would land after the -3 V recompute and
  silently kill the outtake. In the button poll the cancel is immediate, the
  zeros land first, and the execute-phase reverse writes win.
- Everything else is unchanged: the outtake stays momentary with no subsystem
  ownership; the intentionally preserved Shooting-over-reverse write conflict;
  RT local reverse (which does not stop the latch, matching the old
  held-LT-plus-RT coexistence); preload/CANrange indexing inside
  `IntakeCommand`; the INTAKING heartbeat feeding `OperatorStatus` and the
  read-only panel keys; controller priority routing; and the intake deploy
  position behavior (stays extended when collection stops, D-pad left
  retracts).

## Consequences

The operator collects with one tap and stops with a second tap. An outtake tap
mid-collection cleanly swaps the feed path direction; after the outtake the
robot sits idle until LT is pressed again. Enable/disable and controller
handoffs always end with collection stopped, so an operator never finds the
roller running for a reason they did not just command. The panel shows
"Intaking" for as long as the latch actually runs, because the heartbeat comes
from the command's `execute()`.

## Verification boundary

Verified in simulation only (unit/integration tests plus `simulateJava`):
toggle on/off across releases, eject takeover and non-resume, disable and
re-enable, disconnect and priority loss, interruption by Shooting and by
D-pad-left homing, and the poll-phase cancel ordering against the reverse
recompute. Not verified on the real robot: actual motor currents, the physical
feel of a latched collector, and game-piece behavior with the real hopper.
