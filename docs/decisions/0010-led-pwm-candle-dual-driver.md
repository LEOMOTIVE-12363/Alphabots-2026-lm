# 0010 — LED dual-driver prep: PWM / CTRE CANdle switchable by one constant

- Date: 2026-10-23
- Status: Accepted (implementation local on the demo working tree; not yet merged)
- Revised 2026-10-24: onboard LEDs now follow the external display (was: held
  dark), and the send cache was reworked to per-pixel confirmation after Codex
  acceptance confirmed three bounded-send defects — see the revision section.

## Context

The addressable strip's final driver hardware is undecided: it may stay on a
roboRIO PWM output (`AddressableLED`) or move to a CTRE CANdle. What is known
(2026-10-23): the strip is a 12 V WS2811 COB strip; the CANdle, if used, gets
CAN ID 0, but whether it sits on the roboRIO bus or a CANivore bus is open, and
the device has not been checked in Tuner X. Install parameters (PWM port, real
pixel count/density, color order, section wiring) are all still placeholders.

Rather than wait for the hardware decision, both drivers are written now so the
pick is a deploy-time edit, and the light experience (five Día de los Muertos
themes, idle effects, ball-flow layer, Elastic settings and their persistence)
stays shared — it must not be duplicated per hardware.

## Decision

- `LEDConstants.HARDWARE` (enum `PWM` / `CANDLE`, default `PWM`) is the single
  selection constant. Switching = edit that constant → build → redeploy. No
  Elastic hardware toggle, no runtime hot-swap, and never two real devices at
  once: `LEDIO.create()` is the one selection entry and constructs only the
  selected driver (the other hardware object is never opened).
- `LEDIOReal` was renamed `LEDIOPWM`; `LEDIOCandle` was added. The `LEDIO`
  contract is "logical RGB in, brightness already applied": each driver converts
  to the physical channel order exactly once — PWM via a software R/G swap
  (`copyInStripColorOrder`, now covering legacy effects too), CANdle via the
  `StripType` config (`GRB` when the swap flag is set, `RGB` otherwise). The
  CANdle's device brightness scalar is pinned to 1.0 so flash-persisted Tuner
  experiments can never scale brightness a second time.
- CANdle specifics (Phoenix 6 26.3.0, verified against the local jar and CTRE
  docs): external pixels live at device indices 8+ (onboard 0–7 held dark by an
  init paint); external capacity 392 pixels (indices 8–399), clamped with a
  one-time warning; config = `StripType` + `VBatOutputMode.On` (12 V strip
  supply) + `Enable5VRail.Disabled`; `clearAllAnimations()` once at init;
  per-pixel writes use one-shot `SolidColor(startInclusive, endInclusive)`
  control frames.
- Bounded sending: identical frames are skipped (static/Off displays cost zero
  traffic), adjacent same-color pixels coalesce into single frames, and at most
  `CANDLE_MAX_FRAMES_PER_UPDATE` (24) frames go out per
  `CANDLE_MIN_UPDATE_INTERVAL_SEC` (50 ms) tick — worst case 480 frames/s,
  ~6% of the roboRIO bus's 1 Mbit/s; a 60-px all-distinct frame fully refreshes
  in 3 ticks (150 ms), continuing where it left off because the firmware
  retains painted ranges. Smoothness at these defaults is a preview proposal,
  to be judged on the real strip.
- Failure isolation: every CANdle call returns a `StatusCode`; faults log on
  first occurrence, at most every 10 s, plus one recovery note — never an
  exception, never a silent fallback to the other driver. SIM runs the same
  logic against the `CANdle.none()` stub (fully offline); REPLAY keeps the
  empty IO (no hardware output, no NT, no settings writes).

## Consequences

- All not-yet-known install parameters live in `LEDConstants` with explicit
  TODO markers (see the fill-in checklist in AGENTS.md 2026-10-23 LED
  dual-driver section); "one-constant switching" presumes both drivers'
  parameters have been filled in.
- The R/G swap now applies to legacy effects too (previously only theme
  patterns); with the flag still `false` this changes nothing today.
- Not verified on hardware: real pixel count, color order, animation
  smoothness over CAN, and the 12 V VBat supply compatibility all wait for
  installation. Compilation and fake-transport tests are not hardware
  validation.

## Revision (2026-10-24)

Two changes supersede parts of the original decision:

1. **Onboard LEDs follow the external display** (user decision 2026-10-24,
   superseding "onboard 0–7 held dark"). The driver samples the rendered frame
   at eight evenly spaced strip positions and paints them to device indices
   0–7: same theme, clock, animation phase, ball-flow direction (onboard
   logical 0→7 = intake→shooter — a thumbnail, not a physical ball-path
   position), Off and the same already-applied brightness scaling. No separate
   Elastic settings, no second animation, nothing rescales brightness. Per the
   CANdle User's Guide the firmware compensates the onboard LEDs' color order,
   so the external `StripType` config cannot corrupt onboard colors; the 5 V
   rail is now ENABLED in the device config because a disabled rail turns the
   onboard LEDs off (this replaces the original `Enable5VRail.Disabled`).
   Onboard pixels share the external frame budget and never count toward
   `LENGTH_PIXELS` or `SECTIONS`.

2. **Per-pixel confirmation replaces the whole-frame cache.** Codex's
   independent acceptance (2026-10-24) confirmed three defects in the original
   whole-frame `lastSentPacked` cache: Off skipped after a partial refresh
   (stale cache believed the device dark), failed paints cached as successful
   (never retried), and multi-tick partial refreshes never converged (kept
   re-sending the full budget forever). The driver now tracks, per logical
   pixel (onboard + external), the color the device is believed to show —
   updated only by OK paints. Dirty pixels plan into coalesced runs under the
   same budget/throttle, a rotating cursor keeps continuously-changing frames
   fair, every tick plans from the latest target (no stale-frame queue), and
   any paint failure invalidates all confirmations so recovery triggers one
   bounded full repaint (covers an unnoticed device reset). Budget, throttle
   and coalescing are unchanged; the three acceptance tests were ported into
   `LEDIOCandleTest` unweakened, and Codex's original independent test file
   passes against the fixed driver unchanged.

## Revision 2 (2026-10-24, second acceptance round)

Three further findings (two P1/P2 recovery defects, one semantics defect) reworked
two areas; the per-pixel confirmation model from Revision 1 is unchanged.

1. **Initialization is a tracked state with bounded recovery.** A failed boot-time
   config (device powered after the program, transient startup failure) used to be
   logged and forgotten, leaving power rails / strip type / brightness scalar /
   animation cleanup permanently unapplied. The driver now records whether
   initialization succeeded and retries it at most once per
   `CANDLE_INIT_RETRY_INTERVAL_SEC` (2 s) — and only while the device answers its
   cached status signal (`getVersion().getStatus()`, non-blocking), so an absent
   device never blocks the loop with config timeouts; the config apply carries an
   explicit 0.1 s timeout. Any successful (re-)initialization invalidates the
   confirmation cache so the latest target repaints under the normal budget.

2. **Device resets are recovered from, not just paint failures.** Resets were
   invisible while a static frame sent no paints. Now: Phoenix 6
   `hasResetOccurred()` (client-side, non-blocking) triggers a config re-apply —
   a reset reverts the device config to flash defaults — plus cache invalidation,
   at most once per refresh interval (the interval check runs before the flag is
   read so an early flag is never consumed unhandled); AND, because a reset can
   happen without any signal reaching the driver, the confirmation cache is
   invalidated every `CANDLE_REFRESH_INTERVAL_SEC` (2 s) as a bounded keepalive,
   so the latest target always repaints within the normal frame budget. Static
   displays are therefore refreshed at a low bounded rate, not strictly
   zero traffic. Off and brightness-0 stay dark on both segments.

3. **The onboard thumbnail is a logical ball-path render, not physical
   sampling.** Physical-index sampling assumed external pixel 0 = intake and
   ignored `reversed` sections, split side panels and decoration runs. The shared
   render pipeline (`LED.renderThemeInto` / `renderLegacyInto`, extracted from the
   single existing implementation) now renders the thumbnail at eight normalized
   u positions on a fixed non-reversed BALL_PATH section (onboard 0→7 =
   intake→shooter) with the same theme, clock, animation phase, ball-flow state
   and single brightness scaling, delivered through the new
   `LEDIO.writeOnboardBuffer` (PWM and replay keep the no-op default). The
   external wiring can never change the onboard logical direction. Both segments
   still share one frame budget and the external `StripType` still cannot corrupt
   onboard colors (firmware compensates).

## Revision 3 (2026-10-24, hardware selection)

The user has now selected CANdle as the strip driver. This supersedes only the
original "hardware is undecided" context and the default-`PWM` selection:
`LEDConstants.HARDWARE` is now `Hardware.CANDLE`.

PWM remains available as a compile-time fallback through the same one-constant
selection mechanism; its driver, factory branch and tests are retained. There is
still no runtime hot-swap, Elastic hardware toggle, automatic fallback after a
CANdle fault, or simultaneous construction of both real devices.

This selection does not verify the installation. CAN ID 0 remains the configured
value pending Tuner X inspection, and bus `rio`, pixel geometry, color order,
section wiring, firmware, power rails, CAN smoothness and real-roboRIO timing all
remain subject to physical verification. The implementation remains local and
unmerged, as recorded in the status above.

## Revision 4 (2026-10-24, configured strip length)

A partially lighting strip exposed that `LENGTH_METERS` was still 3.0 m, so only
60 pixels were rendered and addressed at the current 20 px/m density. The user
then set the required total configured length to 10.0 m. `LENGTH_PIXELS` is now
200, below the CANdle's 392-external-pixel capacity; PWM and all other install
parameters are unchanged.

The regression `configuredTenMeterStripPaintsAllTwoHundredPixels` failed at
external pixel 60 before the change and now verifies color delivery to every
one of the 200 external pixels. Physical density, full-strip illumination,
color order, power delivery and smoothness still require on-strip verification.

## Revision 5 (2026-10-24, pre-merge recovery review)

Pre-merge review found three recovery/logging defects and two test weaknesses.
Initialization now records both success and failure, so a failed reset-triggered
reinitialization enters the bounded retry path. The periodic keepalive now
re-applies configuration on responsive devices as well as invalidating color
confirmations, so an undetected reset cannot leave flash-default configuration
in place. Fault/recovery logging is aggregated per update batch, preventing a
mixed OK/failed batch from emitting repeated recovery messages.

New regressions cover failed reset reinitialization, undetected-reset config and
display recovery, and mixed-result log throttling. The 10 m test now refreshes
200 distinct colors under the 24-frame tick cap, and the reversed-wiring sanity
check compares actual pixel contents. PWM remains untouched as the fallback.

## Revision 6 (2026-10-24, restore 3.0 m configured length)

The user decided to restore the original 3.0 m configured strip length, ending
the interim 10.0 m configuration from Revision 4. `LENGTH_METERS` returns to
3.0, so the derived `LENGTH_PIXELS` is 60 again; the 20 px/m density, the
section map, and the PWM fallback are unchanged.

The 10 m regression test became `configuredThreeMeterStripPaintsAllSixtyPixels`:
it drives the CANdle with `LEDConstants.LENGTH_PIXELS` pixels, asserts every
tick stays within the 24-frame budget, and verifies color delivery to all 60
external pixels within the expected 3-tick full refresh (8 onboard + 60
external runs). The measured strip length and pixel density still require
on-strip verification; this revert is a configuration change, not a hardware
acceptance.

## Revision 7 (2026-10-24, reconfirm 10.0 m configured length)

The user reconfirmed a total configured strip length of 10.0 m, superseding the
3.0 m restoration in Revision 6 while preserving Revision 4 and Revision 6 as
history. `LENGTH_METERS` is 10.0 again; the 20 px/m density and the derived
`LENGTH_PIXELS` formula are unchanged, so the driver addresses 200 external
pixels. This remains below the CANdle's 392-external-pixel capacity. The frame
budget, section map, PWM fallback and all other parameters are unchanged.

The regression `configuredTenMeterStripPaintsAllTwoHundredPixels` again covers
the real 10 m addressing behavior: 200 valid, distinct RGB colors must reach
external device indices 8–207, every tick must stay within 24 frames, and the
actual send-count bound is ceil((8 + 200) / 24) = 9 eligible sends. The 450 ms
figure is only the theoretical throttling time at the configured 50 ms
interval, not a worst-case scheduling bound; with 20 ms scheduling, nine
eligible sends typically take about 540 ms. The RED run on the previous 3.0 m
configuration was observed failing as expected at external pixel 60
(device index 68). Physical density, full-strip illumination, color order,
power delivery and smoothness remain unverified on the real LEDs; the
fake-transport regression is not hardware acceptance.

## Revision 8 (2026-10-24, dashboard-only LED visuals)

The ball-flow and robot/mechanism-state visual requirements above, including
references to ball-flow direction/state in onboard rendering, are superseded by
[0012 — Dashboard-selected LED output only](0012-dashboard-selected-led-only.md).
The hardware driver, installation, persistence mechanism, and
bounded-send/recovery decisions are unchanged; 0012 defines the current visual
and diagnostic semantics.

## Revision 9 (2026-10-25, third acceptance round: non-blocking config, unified color order, shared Chase lap)

Codex's third independent acceptance round (baseline demo `50828e9`) found
three driver defects; all three are fixed here and the round-1/2/3 independent
tests are kept as regressions. None of the earlier revisions' visual semantics
(0012 dashboard-only output, Revision 4/7 onboard logical thumbnail) changes.

1. **Blocking configuration (P1).** Every blocking device call — the 0.1 s
   configurator apply, animation cleanup, init onboard dimming — moved off the
   robot's periodic thread onto a shared single-thread daemon worker
   (`LED-CANdle-config`). The loop thread only makes rate-limited decisions on
   its own clock (init retry every 2 s while the device answers, reset-flag
   re-init, 2 s keepalive) and hands the work to that worker; at most one
   config task is in flight (no stale-task queue). Config and paints serialize
   through a hardware lock: the worker holds it for the whole config pass, the
   loop only `tryLock`s — while a config owns the device the tick sends nothing
   and the dirty state survives to the next tick. Success is recorded only
   after the task truly completes, and the confirmation cache is invalidated
   after every completed pass (success or failure — the display is suspect
   either way), so the latest frame always repaints under the normal budget.
   Startup failure retry, reset recovery, no-signal keepalive, throttling and
   fault-log rate limits are all preserved. Construction still configures
   synchronously once (bounded boot cost, same as before). Independent
   evidence: 100 ms injected config latency previously measured 101–105 ms
   inside `writeBuffer`; after the fix the same injection keeps the periodic
   write under 20 ms (`LEDThirdAcceptanceTest`, ported verbatim, plus the
   in-suite twins covering deferred sends during an in-flight config and the
   failure-then-retry path).

2. **Unified physical color-order contract (P2).** Verified against the
   project's own WPILib 2026.2.1 HAL sources: the roboRIO FPGA natively
   outputs GRB, `AddressableLED` defaults to GRB and converts exactly once for
   any other configured order (`setColorOrder`, added in 2026; sim HAL no-op).
   The old PWM driver relied on the GRB default plus a software R/G swap at
   the opposite polarity from the CANdle's `StripType` mapping, so the two
   drivers wired opposite channel orders at the same flag value. The contract
   is now one statement — `SWAP_RED_GREEN_FOR_WS2811 == true` ⇔ the strip's
   physical channel order is GRB (classic WS2811) — implemented by the PWM
   driver explicitly setting the HAL color order (GRB/RGB, never the default)
   with a logical-RGB identity copy, and by the CANdle driver's existing
   `StripType` GRB/RGB mapping; the conversion happens exactly once, in
   hardware, in both drivers. `LEDColorOrderContractTest` derives the wire
   channel sequence of both drivers through their real entry points for pure
   red/green/blue at both flag values. The constant's value stays `false`
   (= RGB-ordered strip) so the deployed CANdle configuration is byte-identical
   to before; the actual strip order remains an install-time TODO to verify
   (flip the flag if pure red renders as green).

3. **Shared legacy-Chase lap (P2).** The base `LED Pattern = Chase` previously
   derived its lap from the target buffer's own length, so the 8-px onboard
   thumbnail cycled in (8+6)/20 = 0.7 s while the 200-px strip cycled in
   10.3 s — same-clock, different phase, violating the shared-phase thumbnail
   contract. `renderLegacyInto` now defines the lap from the configured
   external length (`LEDConstants.LENGTH_PIXELS` + tail): a reference-length
   target (the strip render and the preview's full-length legacy buffer) keeps
   the exact original per-pixel math, and any other length (the thumbnail)
   samples the same head position at normalized u positions, like the theme
   thumbnail. The theme idle effect "Chase" (`IdleEffect.CHASE`) is a different
   animation and is untouched. Regressions: `LEDThirdAcceptanceTest` (verbatim)
   plus `LEDLegacyChaseSyncTest` (lap/phase/wraparound across lengths, single
   brightness scaling, preview periodicity).

Verification (local, no real strip/CAN): full offline build 31 suites /
312 tests / 0 failures/errors/skipped (baseline 26 suites / 290 + 5
new suites / 22 new tests); bounded
PWM- and CANDLE-configuration `simulateJava` smokes each reached robot-program
startup complete with zero exceptions and zero loop-overrun warnings in the
window (the CANdle sim still uses the `none()` stub — real-bus latency is
covered only by the injected-latency tests). The round-2 recovery tests were
ported with an async-preserving adaptation (await the worker, then drive a few
ticks — assertion strength unchanged) because recovery now runs on the
background worker by design.
