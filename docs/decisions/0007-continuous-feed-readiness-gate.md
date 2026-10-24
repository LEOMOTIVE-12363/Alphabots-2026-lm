# 0007 — Continuous feed readiness gate

- Date: 2026-10-22
- Status: Superseded by [0011](0011-immediate-feed-only-flywheel-zero-gate.md)
  (2026-10-24) for drum/hood/stable-readiness feed gating; see 0011 for the
  behavior retained from this record

> **Superseded on 2026-10-24 by [0011 — Immediate feed with only flywheel-zero gating](0011-immediate-feed-only-flywheel-zero-gate.md).**
> The continuous drum-at-speed, hood-at-angle, and stable-readiness feed gate
> described below is no longer the desired behavior. This includes startup and
> recovery debounce waits, mid-volley pauses on speed/hood/target changes, the
> equivalent Manual gate, and post-UNJAM readiness re-gating. The original record
> and amendments are retained below for history only; 0011 defines the immediate-
> feed replacement and the safety, lifecycle, jam/timer, controller, and dashboard
> behavior that remains in force.

## Context

Feeding previously started on a one-time readiness check in SPIN_UP: once
the volley began, nothing re-checked the shot. A mid-volley flywheel or hood
loss — or dropping the Flywheel Speed Limit to 0% mid-demo — kept the
feeder, indexer, and intake roller running, pushing fuel into a shooter that
could not launch it. Manual had its own readiness copy with a hood tolerance
of 0.1 rot (36°) — wider than the hood's whole 0–25° range, so it never
gated.

## Decision

Gate feeding continuously in every feeding state through one shared rule,
`util/ShotReadiness` (flywheel limit > 0 AND drum at speed AND hood at
angle; tolerances are the existing `ShootingConstants` values). Pausing
stops the feed path but keeps driving the drum and hood to target (new
PAUSED state in Shooting); the volley auto-resumes on stable readiness
(0.15 s debounce) without a button re-press, and the retract timer freezes
while paused. UNJAM is never suppressed by the gate — the back-off always
runs to completion — but re-gates through PAUSED before forward feeding
resumes. Manual is unified onto the same rule (SHOOT falls back to PREPARE
on raw-not-ready, re-enters on stable-ready), and the new operator panel
(`util/OperatorStatus`, command-heartbeat based, published to Elastic at
10 Hz) reads the same evaluation, so display and control cannot drift.

## Consequences

Behavior changes, all deliberate: UNJAM no longer resumes feeding directly
but passes the readiness gate first; paused/unjam time no longer counts
toward the retract delay (previously a >0.2 s unjam meant an instant RETRACT
on resume); Manual's PREPARE→SHOOT went from a single sample to the 0.15 s
debounce; Manual's hood tolerance is now the shared 0.02 rot (7.2°). That
hood tolerance is 29% of the 25° range and may be too loose — tightening is
pending real-robot validation on Saturday. No new tuning parameters were
introduced. New read-only NetworkTables keys under /SmartDashboard
(`Robot Action`, `Feed Allowed`, `Feed Blocked Reason`,
`Drum Target Speed (rot/s)`, `Drum Actual Speed (rot/s)`,
`Battery Voltage (V)`, `Controller 1 Connected`) plus the
`Shooting/stableReady` log key are external interfaces consumed by
`elastic-layout-1022.json` and AdvantageScope layouts.

## Amendment — 2026-10-23 acceptance rework

Codex acceptance testing of the merged implementation (d750db7) found five
edge-case failures; all were fixed without changing the approved parameters
(hood tolerance 7.2°, debounce 0.15 s, retract 0.2 s, unjam 0.5 s):

1. **Stop on the detection loop.** SHOOT/RETRACT previously switched to
   PAUSED but only wrote the stop outputs on the next loop; the feed path
   now stops on the very loop readiness is lost (drum/hood still driven),
   in velocity mode and in the A-button current-feed mode alike. Manual
   got the same same-loop stop.
2. **Freeze the retract timer at UNJAM entry**, not when the back-off ends
   — previously a 0.5 s unjam counted toward the retract delay.
3. **Reset the readiness debounce at UNJAM entry and hold it not-ready
   through the back-off** (`ShotReadiness.holdNotReady()`): the WPILib
   kRising `Debouncer` only re-anchors on a baseline sample and its timer
   starts at construction, so a quiet 0.5 s back-off otherwise let the
   first post-unjam ready sample pass the stable window instantly. After
   the back-off the shot must re-earn a full fresh stable window.
4. **The panel's feed decision is reported by the active command**
   (`OperatorStatus.reportFeedDecision`, 150 ms freshness, cleared in
   `end()`), replacing the fixed full-speed panel-side evaluation that
   misreported Manual's lower setpoints as blocked. The panel falls back
   to its own evaluation only when no command decision is fresh; a fresh
   CLEARING_JAM heartbeat overrides to blocked.
5. **The stabilizing window is shown, not bypassed**: the panel reflects
   the command's actual permit, with the dedicated reason
   `Stabilizing shot` while raw-ready but not yet stable — a separate
   panel-side debouncer could reset at different times than the command's
   and drift.

Test-environment note for future tests: in unit tests `Timer`
(`RobotController.getTime`) and `Debouncer` (`MathSharedStore.getTimestamp`)
run on clocks that lag each other by one 0.02 s step, so the 0.2 s jam
debounce and the 0.2 s retract delay never expire on the same loop — a jam
may legitimately trip out of RETRACT, not only SHOOT (the common case on
the real robot too, since a stall usually appears after the retract).
Assertions on the feeding state must accept both feeding states.

## Amendment 2 — 2026-10-23 jam window only counts feeding time

Codex re-verification of 252d653 reproduced a residual edge case: after a
pause of 0.2 s or longer, the first high-current sample after feeding
resumed tripped UNJAM instantly. Root cause: the kRising `Debouncer`
re-anchors its timer only on baseline (clear) samples and lets it free-run
otherwise, and PAUSED fed no samples at all, so paused time silently filled
the 0.2 s jam confirmation window. The earlier record describing this as a
"one loop early" bias was wrong — a single post-pause spike tripped the
unjam, and the same gap existed in SPIN_UP (a real spin-up can take
seconds) and in the post-unjam recovery.

Fix: the jam confirmation window now accumulates only while feeding. A new
`holdJamNotDetected()` feeds the debouncer a clear sample every loop in all
three non-feeding states (`spinUp()`, `paused()`, `unjam()`), the same
pattern as `ShotReadiness.holdNotReady()`; `enterPause()` no longer
recreates the debouncer (re-anchoring every paused loop makes that
redundant), while `initialize()` still starts with a fresh instance. No
tuning parameters changed (jam threshold 90 A, confirm 0.2 s, unjam 0.5 s,
ready debounce 0.15 s, retract 0.2 s). Manual has no jam detection and is
unaffected.

Three regression tests cover it: a single post-pause current spike never
trips UNJAM; sustained high current after a long pause trips only after
the full window; and the post-unjam recovery starts with a clean window
(no residual timing re-trips). Test-ordering note: the current override
must be followed by a manual `Indexer.periodic()` before the next
`execute()`, or the first check reads the stale low current, feeds a clear
sample, and masks a free-running debouncer. Verified by negative control:
all three tests fail against the pre-fix code (UNJAM on the first loop).
