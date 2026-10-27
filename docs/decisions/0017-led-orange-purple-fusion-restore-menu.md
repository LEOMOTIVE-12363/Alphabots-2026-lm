# 0017 — Orange+purple fusion gradient on all themes; menu fully restored; 7 m strip

- Date: 2026-10-26
- Status: Partially superseded by [0026](0026-restore-distinct-led-theme-palettes.md) (unified palettes only; menu restoration and 7 m length retained)
- Supersedes: [0014](0014-slim-led-menu-three-themes.md)

## Context

After PR #52 shipped the three-theme menu, the operator reported that Festival Night showed
"basically all purple, almost no orange" on the robot. Diagnosis confirmed it: the code rendered
the gradient over the configured 10.0 m / 200 px while only ~7 m of strip is installed, so the
visible pixels covered only u ∈ [0, 0.70] — 49% purple-family, 43% muted transition, and the vivid
orange stop `#FF7900` at u=1 landed on LEDs that do not exist. The operator also decided that a
one-way "purple head, orange tail" gradient is not the desired look at all: they want orange and
purple **fused across the whole robot**, alternating band by band. Separately, the operator asked
to fully restore the menu entries removed that same day, then apply the orange+purple scheme to
every theme.

## Decision

- **Strip length**: `LENGTH_METERS` = **7.0** (user-stated installed length; 140 px at the current
  unverified 20 px/m assumption), superseding the 10.0 m value from 2026-10-24. The gradient now
  spans the physical strip.
- **Fusion gradient**: every one of the five Día de los Muertos themes renders the SAME
  orange+purple fusion gradient — alternating stops `#7020D0` (purple) at u = 0, 0.5, 1.0 and
  `#FF7900` (orange) at u = 0.25, 0.75, smoothstep-blended per RGB channel between stops. Both
  colors appear across the full strip at any installed length. The five entries survive as
  operator presets that differ **only in name and recommended idle effect** (Slow Flow / Breathe /
  Chase / Slow Flow / Static); their former per-theme palettes are not retained.
- **Menu fully restored** (reversing 0014): `LED Pattern` is again Off → Marigold Bridge → Solid →
  Rainbow → Chase → Breathe → Festival Night → Papel Fiesta → Alebrije Glow → Candle Memory. The
  legacy standalone patterns, their render paths (`renderLegacyInto` and the shared-lap Chase
  machinery), tuning constants, tests, and the two theme cards in the Elastic `LED Preview` tab
  are restored as they were before the slim. DS1 Back again cycles all five themes; Start remains
  a no-op under Off/legacy patterns. The layout keeps the `elastic-layout-1026.json` name (same
  modification day) with all five theme cards.
- `LEDPattern.fromDashboardName` (added by 0014) is retained as the boot-restore migration seam;
  with the menu restored it now simply resolves every current option, and unknown saved names
  still fall back to the Marigold Bridge default.
- Nothing else changes: idle-effect animations, brightness and power-cycle persistence, PWM/
  CANdle dual-driver selection, the onboard logical thumbnail, preview publishing, and all
  mechanism control are untouched.

## Consequences

- On a 7 m / 140 px strip each color band spans roughly 1.75 m; the strip starts and ends purple
  with four purple↔orange transitions. At other installed lengths the band count stays the same
  (bands scale with the configured length), so the fusion look survives wiring changes — but the
  config must still track the physically installed length to light the whole run.
- The five theme choices are visually identical at rest under the same idle effect; operators who
  want variety pick different idle effects (each theme's recommendation differs). The standalone
  Solid/Rainbow/Chase/Breathe patterns provide the non-orange-purple options.
- Pixel-density (20 px/m) remains an unverified assumption; the real-strip color order, power,
  and smoothness checks are still pending. No robot deploy or real-strip verification occurred
  before this merge.
