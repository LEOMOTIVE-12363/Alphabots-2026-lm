# 0022 — Acceptance fixes: per-press accumulated timeout, persisted timeout setting, real LT edges

- Date: 2026-10-27
- Status: Accepted
- Amends: [0021 — Widen speed tolerance + force-feed timeout](0021-widen-tolerance-force-feed-timeout.md)
  (accumulation semantics) and [0016 — Latched intake toggle](0016-latched-intake-toggle.md)
  (edge implementation, honoring its original no-self-resume promise)
- Amended by: [0024 — Startup-only feed gate](0024-startup-only-feed-gate.md)
  (timeout accumulation retained; ordinary post-release sag no longer creates waiting)

## Context

The 2026-10-27 independent acceptance run against demo `b977a9f` found three
P2 issues: (1) the "accumulated 3 s" force-feed timeout was silently reset by
`clearGateWait()`/Manual's reset branch every time feeding briefly recovered,
so stuttering speed postponed the fallback indefinitely — the opposite of
0021's accumulated-per-press wording; (2) `/Tuning/Shooting/ForceFeedTimeoutSec`
existed as a LoggedTunableNumber with no Elastic widget and no persistence,
violating the repo's new-dashboard-setting rules; (3) the latched intake
restarted when control returned to a pad still holding LT (a "has control AND
pressed" trigger condition regaining truth masquerades as a press), and a later
release could not stop it — contradicting 0016's release-and-repress promise
that the implementation notes had already flagged as inherited platform
behavior.

## Decision

1. **Per-press accumulation (0021 amended).** The gate-wait timer accumulates
   across the whole press at a positive flywheel limit — SPIN_UP and
   below-speed pauses count, feeding/0%/UNJAM stretches pause it WITHOUT
   clearing it. Only `initialize()` (fresh press) resets the accumulator. The
   ±3.0 rot/s tolerance, 0.15 s stable window, latch-until-release, and the
   0%-never-counts/never-crossed rules are unchanged, for both Shooting and
   Manual.
2. **Persisted dashboard setting.** The timeout moves from the
   LoggedTunableNumber to `DashboardControls` as
   `/SmartDashboard/Force Feed Timeout (s)` (default 3.0 s), riding the
   existing DashboardSettingsFile save/restore path. Values are sanitized to
   [0.5, 10] s with NaN/inf falling back to the default, so a stray edit can
   neither disable the fallback (0 s) nor freeze the robot out. The demo tab of
   `elastic-layout-1027.json` (renamed from 1026 per the MMDD rule) carries a
   numeric Text Display with submit button, like the percentage inputs.
3. **Real LT edges for the latch (0016 honored).** The toggle is no longer a
   scheduler-trigger edge on "has control AND pressed". RobotContainer polls
   the raw LT state every loop (after the scheduler, from `robotPeriodic`):
   only a physical rising edge observed while the pad is connected, in control,
   and the robot is enabled toggles the latch. A latch torn down while LT was
   held (priority loss, disconnect, disable, LB, interruption — or the poll's
   own toggle-off, which is behavior-neutral) sets an awaiting-release flag on
   that port; the next toggle needs an observed release first. While a pad is
   disconnected its button reads are garbage, so its edge state is frozen and
   the reconnect loop only resynchronizes the baseline — a hold spanning the
   disconnect is not a press either. All other LT semantics are unchanged:
   press-on/press-off, port-1 priority, LB cancels-then-ejects with no
   auto-resume, RT untouched, disable/conflict teardown unchanged, and no
   other button's takeover behavior touched.

## Retained interfaces

- No NetworkTables key names changed except the timeout moving from
  `/Tuning/Shooting/ForceFeedTimeoutSec` (deleted with the tunable) to the
  persisted `/SmartDashboard/Force Feed Timeout (s)`; the layout carries the
  matching widget. Log keys `Shooting/forceFeed` and `Manual/forceFeed` are
  unchanged.
- LED design is untouched: five themes, the orange/purple fusion palette,
  7 m configuration, menu, and previews are exactly as accepted.

## Verification (local; RED first, then green)

- RED on the unmodified tree: `accumulatedWaitSurvivesBriefFeedingRecovery`
  (Shooting + Manual; 1.8 s wait → 0.4 s feeding → 1.8 s wait must force-feed)
  and `intakeLatchDoesNotRestartWhenControlReturnsWhileHeld` /
  `intakeLatchDoesNotRestartOnReconnectWhileHeld` all failed as reproduced by
  the acceptance report; the timeout-persistence RED is the acceptance run's
  own evidence (its 4.5 s restore test failed on this code).
- GREEN: 32 suites / **332 tests / 0 failures / 0 errors / 0 skipped**
  (326 baseline + 6 new: two accumulated-wait, two LT-edge, one persistence
  round-trip/sanitize with 4.5 s restore, one command-uses-the-dashboard-value
  proving a 0.6 s entry force-feeds after ~30 loops rather than 3 s). Layout
  contracts updated for the 22-widget demo tab. 65 s `simulateJava` smoke:
  startup complete, no exceptions, one pre-existing loop-overrun warning.
- Not verified on the real robot: actual standing error, timeout feel, and
  the LT edge behavior with physical pads (the state machine is exercised only
  through DriverStationSim).

## Consequences

A stuttering flywheel can no longer push the fallback forever; the operator
now has an on-robot knob for the timeout that survives power cycles. The LT
latch now honors 0016's promise literally; the cost is a per-loop poll in
`robotPeriodic` (two boolean reads plus an isScheduled check) and the
conservative rule that a press made while its pad is disconnected only counts
after the next real edge.
