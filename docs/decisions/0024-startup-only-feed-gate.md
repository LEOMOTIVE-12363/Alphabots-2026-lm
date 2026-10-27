# 0024 — Release the speed gate at startup instead of cycling feed on every sag

- Date: 2026-10-27
- Status: Accepted
- Amends: [0020 — Flywheel-at-speed feed gate](0020-flywheel-at-speed-feed-gate.md)
  (removes continuous mid-volley gating while retaining startup and recovery gates),
  [0021 — Widen tolerance + force-feed timeout](0021-widen-tolerance-force-feed-timeout.md)
  and [0022 — Acceptance fixes](0022-acceptance-fixes-accumulation-persistence-lt-edges.md)
  (the accumulated timeout remains, but ordinary post-release sag no longer creates waiting)

## Context

The operator reported visibly stuttering shooting at Flywheel Speed Limit 25%,
50% and 75%. The percentage scales only the Drum target; ordinary RB feeding
still commands Indexer and Feeder at fixed speeds. Source tracing found two
independent possible stutter mechanisms that must not be conflated:

1. The 0020 continuous gate stopped feeding on one out-of-window measurement,
   then required a fresh 0.15 s stable window. Ball loading can therefore create
   a relaxation cycle: feed -> drum sag -> immediate stop -> unload/recover ->
   stable-window delay -> feed again. The 3 s force-feed latch eventually bypasses
   this path, but short presses and the time before the latch still stutter.
2. UNJAM is independent of that latch. Indexer current above 90 A for 0.2 s
   commands a 0.5 s reverse before the speed gate, so repeated high current can
   also produce a forward/reverse/stop cycle. There is no real-robot wpilog in
   the repository, so the field frequency of this second path is unproven.

Changing PID gains, the current threshold or feed speeds without measurements
would hide the distinction and could create a different mechanical problem.

## Decision

Make the measured speed gate a **startup/recovery permit**, not a per-loop
mid-volley feed switch, for Shooting (velocity and 50 A modes) and Manual.

- A fresh press still requires measured Drum speed within the existing
  ±3.0 rot/s tolerance for the existing 0.15 s stable window before feeding.
- Once earned, `feedGateReleased` remains true through ordinary mid-volley speed
  sag and nonzero target edits. Drum and Hood keep tracking their targets while
  the feed path continues; no SHOOT/RETRACT -> PAUSED transition is driven by
  `rawReady` alone.
- A 0% Flywheel limit still stops feeding immediately and revokes the permit.
  Returning to a positive value must re-earn the stable window, except when the
  existing force-feed latch is active. Force-feed never crosses 0%.
- UNJAM also revokes the normal permit and re-gates its recovery through the
  stable window. Its 90 A / 0.2 s confirmation, 0.5 s reverse, outputs and
  priority are unchanged. An active force-feed latch still bypasses only the
  post-unjam speed gate, never UNJAM itself.
- `feedGateReleased` is distinct from `forceFeedActive`: a normal at-speed
  release is not logged as a timeout override. A fresh press resets both.
- The existing accumulated force-feed timeout and its persisted Elastic setting
  remain. Startup and below-speed recovery waiting count; feeding, 0% and UNJAM
  pause the accumulator without clearing it. With no more sag-induced pauses,
  the timeout normally matters at startup and after a 0%/UNJAM recovery.
- `drumReady`, `stableReady` and `hoodReady` remain truthful measurements and
  diagnostics. No NetworkTables key, Elastic widget, motor gain, target speed,
  feed speed, current threshold or controller binding changes.

## Consequences

Initial feeding still cannot push a full hopper against a flywheel that has not
spun up, but normal ball-load speed variation no longer manufactures a
stop/start cycle. A sustained severe speed loss after release also no longer
stops feeding by itself; shots may be weak and a real stall must be handled by
UNJAM, 0%, or button release. This is the deliberate smoothness/safety trade-off
of making the gate startup-only.

This change does **not** claim to cure repeated UNJAM or an electrical/mechanical
problem. If the deployed robot still stutters, inspect `Shooting/state`,
`indexerJammed`, Drum target/actual, Indexer leader stator current and battery
voltage in the real log. Repeated `Clearing jam` requires a separate,
measurement-driven jam-classification/back-off fix.

## Verification

- Baseline before edits: 32 suites / 334 tests / 0 failures/errors/skipped.
- RED on the old implementation: exactly six new behavior contracts failed —
  post-release velocity-feed sag, post-release 50 A sag, Manual sag, Shooting
  target edit, Manual target edit, and the real RB binding target edit. The
  other 99 selected tests passed.
- GREEN after implementation: the same four suites passed 105/105, including
  startup gating, 0% recovery, post-UNJAM recovery, force-feed and lifecycle
  behavior. Full `build --offline --rerun-tasks`: 32 suites / 334 tests /
  0 failures/errors/skipped.
- Simulation and injected measurements verify command behavior only. They do
  not model real Drum acceleration, ball-load current, or the physical jam
  waveform. Real-robot deployment and the 25/50/75% comparison remain pending.
