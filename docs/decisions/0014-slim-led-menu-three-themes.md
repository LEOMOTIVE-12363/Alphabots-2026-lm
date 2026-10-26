# 0014 — Slim the LED menu to three themes plus Off

- Date: 2026-10-26
- Status: Accepted

## Context

The `LED Pattern` chooser had grown to ten options: Off, four legacy standalone
patterns (Solid, Rainbow, Chase, Breathe), and five Día de los Muertos themes
(Marigold Bridge, Festival Night, Papel Fiesta, Alebrije Glow, Candle Memory).
For the demo robot the operator wants a short, deliberate Coco-style color
menu. The user decided the final menu is **three Día de los Muertos themes
plus Off**, one of which must be orange + purple; the standalone legacy
patterns and the unkept themes are removed to shrink the choice. The
independent `LED Idle Effect` animation selection (Static / Breathe / Slow
Flow / Chase / Color Step) is untouched — this decision trims color themes
only. It does not change [0012](0012-dashboard-selected-led-only.md)
(dashboard-selected appearance), [0010](0010-led-pwm-candle-dual-driver.md)
(hardware drivers), or [0013](0013-elastic-led-theme-previews.md) (preview
publishing); it narrows the option set those decisions operate on.

## Decision

- The pattern menu is exactly **Off → Marigold Bridge (default) → Festival
  Night → Alebrije Glow**. Marigold Bridge keeps its stops (`#FF6500 → #FFBC24
  → #FF9500`, recommended Slow Flow) and Alebrije Glow keeps its stops
  (`#5636FF → #00D6B2 → #FF248C`, recommended Slow Flow).
- **Festival Night is the required orange + purple theme.** Its former rose
  middle stop `#FF167A` is removed, leaving a clean two-stop gradient
  `#7020D0 → #FF7900` with the existing Breathe recommendation. The theme name
  is unchanged so an existing saved "Festival Night" restores directly into
  the new gradient.
- The legacy standalone patterns Solid, Rainbow, Chase, Breathe and the themes
  Papel Fiesta, Candle Memory are deleted from `LEDPattern`/`LEDTheme`, and
  their render paths (`renderLegacyInto`, the shared-lap Chase machinery) and
  tuning constants are removed. Off is the only non-theme pattern and renders
  black through the same clear path for the strip, the CANdle onboard
  thumbnail, and the Elastic preview.
- Boot-restore migration: saved Off restores Off; kept theme names restore as
  themselves; every removed name — legacy patterns and deleted themes alike —
  resolves to the Marigold Bridge default via `LEDPattern.fromDashboardName`
  with a console warning. The next periodic persists the effective value, so a
  restart can never resurrect a deleted option. Unknown or corrupt settings
  keep the pre-existing safe fallback.
- DS1 Back cycles only the three themes (Off is not in the cycle; it stays
  dashboard-selectable). DS1 Start still cycles the five idle effects while a
  theme is selected and is a complete no-op under Off — it never lights the
  strip. Idle-effect settings and their persistence are unchanged.
- The Elastic `LED Preview` tab keeps the current-selection card and one card
  per remaining theme (three); the two removed theme cards are deleted and the
  layout is renamed `elastic-layout-1026.json` per the dated-layout rule.
  Preview publishing keeps its topics, sample counts, and bounded cadence.
- No animation timing, brightness, persistence, color-order, or hardware
  behavior changes. PWM/CANdle dual-driver selection, the onboard thumbnail,
  and all recovery machinery are untouched.

## Consequences

- Operators see a four-entry menu; the DS1 theme cycle wraps after three
  presses. Anyone who saved a removed option silently lands on Marigold
  Bridge at boot (logged), which is also what a corrupt settings file does.
- Tests pinning legacy patterns, the legacy Chase thumbnail sync, and the two
  removed themes were deleted with their features; the CANdle acceptance
  suites unrelated to rendering (retry interval, non-blocking config) remain.
  New regressions cover the four-option menu, the three-theme cycle, Off
  no-op behavior, and the removed-name migration.
