# 0026 — Restore distinct palettes for the five LED themes

- Date: 2026-10-27
- Status: Accepted
- Supersedes: the unified-palette decisions in [0017](0017-led-orange-purple-fusion-restore-menu.md), [0018](0018-led-fusion-30cm-stripes.md), and [0019](0019-led-deeper-fusion-orange.md)

## Context

The operator observed that every Coco theme looked the same: purple and orange. Source inspection confirmed that all five `LEDTheme` entries called the same `fusionStops()` method. Under the same idle effect their rendered colors were pixel-identical; only their names and recommended effects differed. This was the intentional result of 0017–0019, not a hardware color-order fault, but it no longer matches the desired operator experience. The operator asked to fix the duplicate palettes.

## Decision

Restore the exact five independent palettes used immediately before 0017 (source revision `715cd63`), without inventing new colors or retuning them:

- **Marigold Bridge**: `#FF6500` → `#FFBC24` at u=0.62 → `#FF9500` at u=1.
- **Festival Night**: `#7020D0` → `#FF167A` at u=0.70 → `#FF7900` at u=1.
- **Papel Fiesta**: `#FF7900` → `#FF167A` at u=0.33 → `#00C9AD` at u=0.67 → `#FFD040` at u=1.
- **Alebrije Glow**: `#5636FF` → `#00D6B2` at u=0.60 → `#FF248C` at u=1.
- **Candle Memory**: `#FF9500` → `#FFC15A` at u=1.

The shared `fusionStops()` generator and its purple/orange stripe constants are removed. Theme names, dashboard ordering, recommended idle effects, smoothstep RGB interpolation, controller cycling, persistence, the 7.0 m configured length, CANdle/PWM drivers, and all mechanism behavior remain unchanged. No NetworkTables keys or Elastic widgets change.

The external strip, onboard thumbnail, current-selection preview, and five fixed preview cards continue to use the same `LEDTheme.colorAt` path. Consequently, all of them show each theme's own palette without separate preview-only color definitions. Saved selections need no migration because they are stored by theme name.

## Verification

- `LEDThemeTest` asserts every historical design stop literally and verifies that the five sampled palettes are distinct. These tests failed on the unified implementation before the production change.
- `LEDBehaviorTest` checks Candle Memory's restored `#FF9500` first pixel on both the external strip and onboard thumbnail.
- Full build and desktop simulation are required before merge. Real-strip hue response still requires a fresh robot deploy; this change does not claim physical LED, CAN, power, or color-order verification.

## Consequences

Operators can again distinguish all five themes by color as well as by their recommended animations. The previous real-strip observation that `#FF7900` can read yellow-green remains an installation/color-response check item; this restoration deliberately recovers the approved historical palettes rather than silently introducing another color-tuning change.
