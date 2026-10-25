# 0013 — Elastic LED theme previews

- Date: 2026-10-25
- Status: Accepted

## Context

The `LED Pattern` and `LED Idle Effect` choosers only expose text names, so an
operator picking between the five Día de los Muertos themes cannot tell the
color palettes or animations apart before committing a selection. The team
asked for in-dashboard previews: one large card following the current
selection and five compact cards, one per theme, each playing that theme's
recommended idle effect. The previews are a selection aid, not a monitor of
real-strip output, and must not be interrupted by mechanism activity or dimmed
by the real brightness setting ([0012](0012-dashboard-selected-led-only.md)
keeps the real display independent of robot state; the previews share that
independence).

Elastic's native `Multi Color View` widget renders a `string[]` of `#RRGGBB`
colors, which the robot can republish at a bounded rate to animate.

## Decision

- Add a package-private `LEDPreview` updated from `LED.periodic()` after the
  hardware buffers are written. It publishes six read-only NetworkTables
  string-array topics under `/SmartDashboard/LED Preview/`: `Current
  Selection` (64 samples) and one card per theme named by its dashboard name
  (32 samples each). No new persistent settings are introduced.
- Previews reuse the single render pipeline — `LED.renderThemeInto` /
  `renderLegacyInto` — with one logical non-reversed `BALL_PATH` section per
  preview buffer, the same animation clock (`Logger.getTimestamp()`), and the
  same effect/theme math as the real strip. Legacy patterns render at full
  `LEDConstants.LENGTH_PIXELS` resolution and are then endpoint-downsampled so
  per-pixel legacy math (rainbow span, chase comet) is preserved.
- All previews render at a fixed reference brightness (scale 1.0); the
  operator's `LED Brightness (%)` never scales them, so a zero real brightness
  cannot blank the selection aid. Effect envelopes (the animation's own light
  and dark phases) still apply because they are part of the render math.
- The current-selection card follows the actual chooser state, including
  legacy patterns: `Off` publishes all `#000000`; Solid/Rainbow/Chase/Breathe
  preview through the legacy renderer. The five theme cards always show their
  theme with its recommended idle effect (`LEDTheme.getRecommendedEffect()`),
  independent of the current selection, and each card's widget title names the
  theme and its recommended effect.
- Publishing is bounded: a 10 Hz scheduled tick, immediate extra publishes on
  selection changes, per-pixel delta formatting (unchanged pixels are not
  re-formatted), static unchanged cards are not republished, and all buffers
  are pre-allocated. No threads are added. Real animation speed is unchanged —
  the previews sample the same ongoing animation, they do not re-time it.
- Previews never feed back: they use dedicated buffers, never touch the
  external-strip or onboard buffers, chooser state, or the settings file, and
  mechanism action (intake, shooting, eject) cannot alter them. They publish
  while the robot is Disabled so themes can be picked before a match; REPLAY
  publishes nothing (same gate as the other LED status keys).
- The Elastic layout gains an `LED Preview` tab holding the two existing
  choosers (moved to native `ComboBox Chooser` widgets on both tabs they
  appear on), the large current-selection card titled with a
  preview-not-real-brightness disclaimer, and the five labeled theme cards.
  The layout file stays `elastic-layout-1025.json` (same-day change, the
  MMDD rename rule does not advance). Screen colors do not verify real-strip
  color order, brightness, power, or section mapping.

## Consequences

- Selecting a theme still auto-recommends its idle effect and the operator can
  override it; the large card reflects the override, the fixed cards do not.
- The six `Multi Color View` cards under `/SmartDashboard/LED Preview/` (one
  current-selection card plus five fixed theme cards) and the tab are the only
  new external interfaces; `TeleopIntegrationTest`'s demo-tab
  contract is unchanged because the preview widgets live on the new tab.
- Screen previews require the robot program or a simulation running and
  Elastic connected to its NetworkTables; opening the layout file alone shows
  no preview data.
