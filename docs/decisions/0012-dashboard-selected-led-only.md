# 0012 — Dashboard-selected LED output only

- Date: 2026-10-24
- Status: Accepted

## Context

Earlier LED behavior combined the operator's Elastic theme and animation
selection with ball-flow and robot/mechanism-state visuals. Intake, shooting,
eject, and reverse could change the displayed flow or direction, and the
`Follow Ball Flow` toggle controlled that coupling. The operator now needs one
predictable display: the same dashboard-selected appearance in Enable, Disable,
idle, and every mechanism action. This decision supersedes the earlier
ball-flow and robot-state visual requirements, including the visual portions of
[0010](0010-led-pwm-candle-dual-driver.md); it does not replace the hardware
driver or mechanism-control decisions.

## Decision

- The visible LED pattern, effect, and brightness are governed only by the
  Elastic selections `LED Pattern`, `LED Idle Effect`, and `LED Brightness (%)`.
  These settings retain their existing power-cycle persistence. No replacement
  LED setting is introduced.
- Enable, Disable, idle, intake, shooting, and eject do not select a different
  pattern, color, direction, or brightness. There is no mechanism-driven
  ball-flow layer or robot-state LED override on either the external strip or
  the CANdle onboard LEDs. The render path does not read `BallFlowTracker` or
  `DriverStation`, so neither ball-flow tracking nor robot enable/disable state
  is an input to LED output.
- Time remains an input to the selected animation. Slow Flow, Chase, Breathe,
  and other user-selected animated effects continue to play; static selections
  remain static.
- The `Follow Ball Flow` dashboard toggle is removed. It no longer enables or
  disables any visual coupling.
- `LED Effective State` and `LED Flow Direction` remain read-only mechanism
  diagnostics used by the operator panel. They do not describe the actual LED
  effect or direction and must not be interpreted as the current light output.
- Controller routing, button bindings, reverse recompute and cleanup, command
  behavior, the intentionally preserved Shooting-versus-reverse write conflict,
  and `OperatorStatus` / operator-panel behavior are unchanged. In particular,
  the reverse heartbeat described in
  [0008](0008-dual-controller-demo-priority.md) continues to feed
  `OperatorStatus` and the operator panel, but no longer feeds an LED ball-flow
  layer.

## Consequences

Robot and mechanism state changes no longer produce LED state changes. The
operator sees the same selected style throughout Enable, Disable, idle, and
mechanism actions, while the chosen animation still advances with time.
Dashboard diagnostics and actual LED output are intentionally decoupled: a
mechanism diagnostic such as a flow direction does not imply a matching light
animation.

The persisted Elastic controls, CANdle/PWM compile-time driver selection,
hardware configuration, single brightness scaling, and bounded-send/recovery
behavior remain governed by their existing decisions. Only the visual coupling
is removed.

## Verification boundary

No real-strip verification is claimed by this record. Software verification can
establish that robot and mechanism transitions leave the dashboard-selected
output unchanged and that selected animations continue to advance with time,
but it cannot establish physical color order, brightness response, full-strip
illumination, CAN/PWM behavior, animation smoothness, power delivery, or actual
hardware behavior while enabled and disabled. Those checks still require the
real LEDs and robot.
