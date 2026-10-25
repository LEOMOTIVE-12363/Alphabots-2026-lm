package frc.robot.subsystems.LED;

/**
 * Constants for the LED strip subsystem — the single tuning surface for the Day of the Dead themes,
 * idle effects, and the strip hardware selection. All animation numbers are preview proposals from
 * the design preview, to be tuned on the real strip once installed; distances are in normalized
 * ball-path units (0 = intake entrance, 1 = shooter top), times in seconds.
 *
 * <p>Hardware selection: the strip can be driven either from a roboRIO PWM output or from a CTRE
 * CANdle. Both drivers exist and compile; {@link #HARDWARE} is the only constant that picks between
 * them, so switching hardware is a one-line edit followed by rebuild and redeploy (no runtime
 * switching, no simultaneous real devices — the unselected driver is never constructed). Fill in
 * BOTH drivers' install parameters below before relying on the one-line switch.
 */
public final class LEDConstants {
  private LEDConstants() {}

  /** Which physical driver the strip runs on. Switch by editing {@link #HARDWARE} alone. */
  public enum Hardware {
    /** roboRIO PWM output (AddressableLED) driving the WS2811 data line directly. */
    PWM,
    /** CTRE CANdle on the CAN bus driving the strip from its VBat (12 V) output. */
    CANDLE
  }

  // The user selected CANdle on 2026-10-24. PWM is retained only as a fallback selected by editing
  // this constant, rebuilding, and redeploying; installation parameters still require verification.
  public static final Hardware HARDWARE = Hardware.CANDLE;

  // ---- Strip geometry (shared by both drivers) -----------------------------------------------

  // Total configured strip length is 10.0 m (user decision 2026-10-24). With the current 20 px/m
  // density, the driver addresses 200 pixels across the full run.
  public static final double LENGTH_METERS = 10.0;

  // TODO(strip install): pixel density is NOT confirmed. The listing's "720 LEDs/m" counts LED
  // chips, not controllable pixels, and the 5 cm cut marks alone do not prove the per-meter pixel
  // count. 20 px/m is a placeholder pending a physical count of one meter of the installed strip;
  // correct it together with LENGTH_METERS above.
  public static final double PIXELS_PER_METER = 20.0;

  public static final int LENGTH_PIXELS = (int) Math.round(LENGTH_METERS * PIXELS_PER_METER);

  // ---- PWM driver (used only when HARDWARE = PWM) --------------------------------------------

  // TODO(strip install): confirm which roboRIO PWM port (silkscreen 0-9) the green data wire is
  // plugged into. Placeholder until then.
  public static final int PWM_PORT = 0;

  // TODO(strip color order): WS2811 strips vary in R/G channel order; flip to true if theme colors
  // look red/green-swapped on the real strip. Both drivers consume this flag exactly once — the
  // PWM driver software-swaps the channels, the CANdle driver configures GRB strip type instead —
  // and it applies to every pattern, legacy effects included.
  public static final boolean SWAP_RED_GREEN_FOR_WS2811 = false;

  // ---- CANdle driver (used only when HARDWARE = CANDLE) --------------------------------------

  // CAN ID confirmed by the user on 2026-10-23. TODO(candle install): the device has NOT been
  // verified in Tuner X — check its actual ID/firmware there before first deploy.
  public static final int CANDLE_CAN_ID = 0;

  // TODO(candle install): bus not decided yet — "rio" is the roboRIO CAN bus; if the CANdle is on
  // a CANivore, put that bus's name (as shown in Tuner X) here instead.
  public static final String CANDLE_CAN_BUS = "rio";

  // The CANdle's 8 onboard LEDs follow the external strip as a LOGICAL ball-path thumbnail (user
  // decision 2026-10-24; second acceptance round 2026-10-24 refined it from physical sampling):
  // the shared render logic evaluates the same theme, clock, animation phase and brightness at
  // eight normalized u positions (onboard 0->7 = intake->shooter, u = k/7) — the
  // external wiring order, per-section `reversed` flags and decoration sections can never change
  // the onboard logical direction. The driver receives these colors through
  // LEDIO.writeOnboardBuffer; they share the external strip's frame budget below and do NOT count
  // toward LENGTH_PIXELS or SECTIONS.
  public static final int ONBOARD_THUMBNAIL_PIXELS = 8;

  // Bounded-send tuning. Each CANdle SolidColor request is one one-shot CAN frame painting one
  // contiguous same-color run, so the per-second frame count is capped by these two constants:
  // worst case = CANDLE_MAX_FRAMES_PER_UPDATE / CANDLE_MIN_UPDATE_INTERVAL_SEC frames/s
  // (default 24 / 0.050 s = 480 frames/s, roughly 6% of the roboRIO bus's 1 Mbit/s). A full
  // refresh of an all-distinct-pixel frame needs at most ceil((8 onboard + pixels) / 24)
  // eligible sends (8 + 200 px => 9), each capped at 24 frames. 450 ms is only the theoretical
  // throttling time at the configured 50 ms interval, not a scheduling upper bound; 20 ms
  // scheduling typically takes about 540 ms. These tuning values are preview proposals pending
  // real-strip smoothness checks.
  public static final double CANDLE_MIN_UPDATE_INTERVAL_SEC = 0.050;
  public static final int CANDLE_MAX_FRAMES_PER_UPDATE = 24;

  // Recovery tuning (second acceptance round 2026-10-24). Static displays are NOT strictly
  // zero-traffic: every CANDLE_REFRESH_INTERVAL_SEC a responsive device has its configuration
  // re-applied and per-pixel confirmation invalidated, then the latest target is repainted within
  // the normal frame budget — a bounded keepalive that restores config and display after a device
  // reset even when no reset signal reaches the driver. A detected reset additionally re-applies
  // the full device setup, since a reset reverts it to flash defaults. Failed initialization is
  // retried at most once per CANDLE_INIT_RETRY_INTERVAL_SEC, and only while the device answers its
  // status signal (a cached, non-blocking read), so an absent device never blocks the loop with
  // config timeouts. All values are preview proposals pending real-strip verification.
  public static final double CANDLE_REFRESH_INTERVAL_SEC = 2.0;
  public static final double CANDLE_INIT_RETRY_INTERVAL_SEC = 2.0;

  // ---- Legacy effect tuning -------------------------------------------------------------------

  // Legacy effect tuning — placeholder values until the strip is on the robot, adjust freely.
  // Hues are WPILib HSV units (0-180).
  public static final int SOLID_HUE = 22; // gold
  public static final int CHASE_HUE = 160; // blue
  public static final int BREATHE_HUE = 22; // gold
  public static final double CHASE_SPEED_PIXELS_PER_SEC = 20.0;
  public static final int CHASE_TAIL_PIXELS = 6;
  public static final double BREATHE_PERIOD_SEC = 2.0;
  public static final double RAINBOW_FULL_CYCLE_SEC = 3.0;

  // Theme switching (operator action): RGB crossfade between the old and new gradient.
  public static final double THEME_TRANSITION_SEC = 0.6;

  // Idle effects (motion on top of the theme gradient, chosen independently of the theme).
  public static final double IDLE_BREATHE_PERIOD_SEC = 4.8;
  public static final double IDLE_MIN_BRIGHTNESS = 0.35; // shared by BREATHE and SLOW_FLOW
  public static final double IDLE_SLOW_FLOW_PERIOD_SEC = 8.0;
  public static final double IDLE_CHASE_PERIOD_SEC = 3.6;
  public static final double IDLE_CHASE_BAND_WIDTH_PATH = 0.25; // total band width
  public static final double IDLE_CHASE_FLOOR = 0.18;
  public static final int IDLE_COLOR_STEP_BLOCK_COUNT = 4;
  public static final double IDLE_COLOR_STEP_PERIOD_SEC = 1.2; // one color-order rotation step
  public static final double IDLE_COLOR_STEP_BLEND_SEC = 0.18; // RGB crossfade between steps

  // TODO(strip wiring): placeholder section map — one BALL_PATH section over the whole strip. Then
  // replace with one entry per SEPARATELY ADDRESSED physical run: absolute start pixel, length in
  // pixels, reversed flag (set when that run is installed intake-at-the-end, so u=0 stays the
  // intake
  // entrance), and role (BALL_PATH for intake/shooter runs, DECORATION for purely cosmetic runs
  // such
  // as the horizontal back panel). Two side panels fed from ONE data line (split copy) display the
  // same data — model them as a single section, not as two address ranges; only runs with their own
  // data wires get their own entries. Final wiring topology is not decided; do not invent one here.
  public static final LEDSection[] SECTIONS = {
    new LEDSection(0, LENGTH_PIXELS, false, LEDSection.Role.BALL_PATH)
  };
}
