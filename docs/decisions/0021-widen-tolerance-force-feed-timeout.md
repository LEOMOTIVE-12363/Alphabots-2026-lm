# 0021 — Widen speed tolerance to ±3 rot/s and add a 3 s force-feed timeout

- Date: 2026-10-26
- Status: Accepted
- Amends: [0020 — Flywheel-at-speed feed gate (speed-only)](0020-flywheel-at-speed-feed-gate.md)
  (gate semantics unchanged; tolerance widened and a timeout latch added)
- Amended by: [0024 — Startup-only feed gate](0024-startup-only-feed-gate.md)
  (ordinary post-release sag no longer pauses or accumulates timeout waiting)

## Context

Hours after 0020 shipped, the field reported the gate never releasing: holding
the shoot button for 10–20 s left the panel on `Spinning Up`, and changing the
Flywheel Speed Limit to any percentage (25/50/…) never made the drum enter the
±1.0 rot/s window. A percentage-independent miss points at a systematic error,
not spin-up time: the drum's velocity loop is pure-P
(`DrumConstants.Drum_CONFIG` slot0: kP 9.5, **kI 0**, kS 3, kV 0.1), so the
steady-state speed error equals the load-current gap divided by kP — a few
amps of gap is a standing 1–2 rot/s error that a 1.5%-of-target window cannot
absorb. The instant-setpoint sim cannot reproduce this (CLAUDE.md: sim gains
do not represent the real robot); the same mechanism likely explains the
original 2026-10-22 "ball stuck in indexer" report that led to 0011.

The user confirmed two changes: a 3 s force-feed timeout (their proposed
value, down from their initial 5 s idea after discussion) and a fix for the
never-ready gate.

## Decision

1. **Widen `SHOOTER_VELOCITY_TOLERANCE_ROTPS` from ±1.0 to ±3.0 rot/s.** A
   preview value chosen to absorb the estimated pure-P standing error
   (1–2 rot/s) with margin; pending real-robot `Drum Target/Actual` readings.
   The stable window (0.15 s) and all gate semantics from 0020 are unchanged.
2. **3 s force-feed timeout (operator-settled), tunable as
   `Shooting/ForceFeedTimeoutSec`.** While a shoot command waits at a positive
   flywheel limit (SPIN_UP and below-speed PAUSED loops), the wait accumulates;
   once it reaches 3 s the volley force-feeds and the speed gate latches OFF
   for the rest of the press (mid-volley sag no longer pauses it), resetting
   only on button release / re-press (or interruption). A 0% limit never
   accumulates and never gets crossed by the latch — 0% is the operator
   deliberately holding the drum stopped. UNJAM keeps running normally inside
   a latched volley, and its back-off still routes through PAUSED, from which
   a latched volley resumes feeding directly. Manual (X) carries the identical
   timeout and latch against its own 50 rot/s target.

## Retained behavior and interfaces

- All bindings, targets, feed speeds, jam parameters, panel key names, and the
  0% same-loop stop are unchanged. The latch is invisible on the panel: a
  force-fed volley reports plain `Feeding` / allowed.
- New log keys `Shooting/forceFeed` and `Manual/forceFeed` document when the
  latch is active.
- The root-cause fix (adding kI to the drum velocity loop, or re-tuning kP/kV)
  is deliberately NOT done here: real-robot control gains are not set from
  desk analysis. Next step once `Drum Target/Actual` deltas are read on the
  robot: tune kI if the standing error is confirmed, and re-tighten the
  tolerance accordingly.

## Consequences

Worst case (drum truly can never reach its target) each press waits 3 s then
empties the hopper at whatever speed the drum has — acceptable for a demo and
strictly better than a 10–20 s hang. Balls fed under the latch may launch
weakly; the tolerance widening should make the latch the rare fallback rather
than the norm. Low flywheel limits (e.g. 25%) with ±3.0 rot/s admit up to ~18%
speed error at release; acceptable for demo use, revisit with real data.

### Verification (local, not on-robot)

Full `./gradlew build`: 32 suites / **326 tests / 0 failures / 0 errors / 0
skipped** (323 baseline + 3 new: force-feed release after the accumulated
wait, latch persistence + fresh-press re-arm, 0% never accumulating — for both
Shooting and Manual variants across the fault/Manual suites). 65 s
`simulateJava` smoke: startup complete, no exceptions. Not verified on the
real robot: actual standing error magnitude, timeout feel, whether ±3.0 makes
the latch rare — check via `Drum Target/Actual` and how often the volley
starts before ~3 s.
