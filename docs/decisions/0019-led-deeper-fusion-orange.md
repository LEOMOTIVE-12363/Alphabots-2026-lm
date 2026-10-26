# 0019 — Deeper fusion orange (#FF6600)

- Date: 2026-10-26
- Status: Accepted
- Amends: [0018](0018-led-fusion-30cm-stripes.md)

## Context

After redeploying the 30 cm fusion stripes, the operator reported the purple `#7020D0` looks
about right but the orange reads "偏黄、有点偏绿" (yellowish, greenish) on the real COB strip —
not the orange they want. `#FF7900` carries a fairly high green weight (G = 121 of 255), and
budget COB strips commonly render the green die relatively brighter, which pushes amber toward
yellow-green.

## Decision

- The shared fusion orange stop changes from `#FF7900` (255, 121, 0) to **`#FF6600` (255, 102,
  0)** — safety orange, green weight cut ~16% — for a clearly orange reading. Purple `#7020D0`
  is unchanged. Because all five themes share the one fusion gradient (0017), this applies to
  every theme entry at once; the 30 cm stripe geometry (0018) is untouched.
- The two design colors now live in named constants `LEDTheme.FUSION_PURPLE` / `FUSION_ORANGE`
  so future tuning edits one line.

## Consequences

- On-screen previews and the real strip show the deeper orange immediately after a redeploy. If
  the real strip still reads yellow-green, the likely culprits are (a) the green die running
  even hotter — drop `FUSION_ORANGE` further (e.g. `#FF5200`) — or (b) an R/G channel-order
  mismatch: flip `SWAP_RED_GREEN_FOR_WS2811` and check whether pure red renders green (the
  documented install-time test; purple reading as blue rather than violet is the companion
  symptom).
- No length, stripe width, animation, brightness, persistence, or menu changes.
