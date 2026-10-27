# 0018 — Fusion stripes every ~30 cm

- Date: 2026-10-26
- Status: Superseded by [0026](0026-restore-distinct-led-theme-palettes.md)
- Amends: [0017](0017-led-orange-purple-fusion-restore-menu.md)

## Context

After 0017 shipped, the operator reported the strip still looked "basically all purple, only the
last bit orange at the back panel", under both Static and the Chase idle effect. That observation
matches the pre-0017 one-way gradient, not the shipped ABAB code — the fusion code was merged and
the deployment directory synced, but the robot itself had not been redeployed, so the robot was
still running the older build. Regardless, the operator also tightened the design: ~1.75 m bands
are too long; they want a color change roughly every **30 cm**.

## Decision

- The unified fusion gradient now alternates purple `#7020D0` / orange `#FF7900` every
  `LEDTheme.FUSION_BAND_METERS = 0.3` m of `LEDConstants.LENGTH_METERS`, still starting and ending
  purple. Stop positions are generated from the configured length, so future length edits keep
  the physical stripe width; the final stripe before the strip end may be shorter when the length
  is not an exact multiple (7.0 m / 0.3 m = 23⅓ → 23 full stripes plus a short purple finish, 25
  stops).
- Everything else from 0017 is unchanged: the ten-option menu, the five identical-color theme
  presets differing only by recommended effect, the 7.0 m configured length, and all animation,
  brightness, persistence, driver, and mechanism behavior.

## Consequences

- On the 7 m / 140 px strip each stripe spans ~6 pixels (assuming the still-unverified 20 px/m),
  so the render is a fine-grained ripple: pure purple/orange at the stops with an RGB blend
  between them. Both colors are visible on every section of the robot under every idle effect.
- Seeing this on the robot requires a redeploy — merging to demo and syncing the deployment
  directory does not deploy code to the roboRIO.
