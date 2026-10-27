package frc.robot.subsystems.LED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A Day of the Dead color theme: an ordered set of gradient stops over the logical ball-path
 * coordinate u (0 = intake entrance, 1 = shooter top). Stops are interpolated per RGB channel with
 * smoothstep easing — never around the HSV wheel — so the designed hues render exactly and no
 * unintended hues appear between stops.
 *
 * <p>The stop RGB values are the approved design inputs; keep them verbatim.
 */
public enum LEDTheme {
  MARIGOLD_BRIDGE(
      "Marigold Bridge",
      IdleEffect.SLOW_FLOW,
      new GradientStop(0.0, 0xFF6500),
      new GradientStop(0.62, 0xFFBC24),
      new GradientStop(1.0, 0xFF9500)),
  FESTIVAL_NIGHT(
      "Festival Night",
      IdleEffect.BREATHE,
      new GradientStop(0.0, 0x7020D0),
      new GradientStop(0.70, 0xFF167A),
      new GradientStop(1.0, 0xFF7900)),
  PAPEL_FIESTA(
      "Papel Fiesta",
      IdleEffect.CHASE,
      new GradientStop(0.0, 0xFF7900),
      new GradientStop(0.33, 0xFF167A),
      new GradientStop(0.67, 0x00C9AD),
      new GradientStop(1.0, 0xFFD040)),
  ALEBRIJE_GLOW(
      "Alebrije Glow",
      IdleEffect.SLOW_FLOW,
      new GradientStop(0.0, 0x5636FF),
      new GradientStop(0.60, 0x00D6B2),
      new GradientStop(1.0, 0xFF248C)),
  CANDLE_MEMORY(
      "Candle Memory",
      IdleEffect.STATIC,
      new GradientStop(0.0, 0xFF9500),
      new GradientStop(1.0, 0xFFC15A));

  /** One gradient stop: path position u and the packed 0xRRGGBB design color. */
  private record GradientStop(double u, int rgb) {
    Color color() {
      return new Color((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
    }
  }

  private final String dashboardName;
  private final IdleEffect recommendedEffect;
  private final GradientStop[] stops;

  LEDTheme(String dashboardName, IdleEffect recommendedEffect, GradientStop... stops) {
    this.dashboardName = dashboardName;
    this.recommendedEffect = recommendedEffect;
    this.stops = stops;
  }

  /** Label shown for this theme on the dashboard chooser. */
  public String getDashboardName() {
    return dashboardName;
  }

  /**
   * The idle effect the designers paired with this theme; pre-selected when the operator switches
   * to the theme, overridable afterwards.
   */
  public IdleEffect getRecommendedEffect() {
    return recommendedEffect;
  }

  /**
   * Smoothstep-interpolated RGB gradient at u (clamped to [0, 1]); u is the ball-path coordinate (0
   * = intake, 1 = shooter).
   */
  public Color colorAt(double u) {
    u = MathUtil.clamp(u, 0.0, 1.0);
    int next = 1;
    while (next < stops.length - 1 && u > stops[next].u()) {
      next++;
    }
    GradientStop a = stops[next - 1];
    GradientStop b = stops[next];
    double span = b.u() - a.u();
    double t = span > 0.0 ? (u - a.u()) / span : 0.0;
    double s = t * t * (3.0 - 2.0 * t); // smoothstep
    Color ca = a.color();
    Color cb = b.color();
    return new Color(
        MathUtil.interpolate(ca.red, cb.red, s),
        MathUtil.interpolate(ca.green, cb.green, s),
        MathUtil.interpolate(ca.blue, cb.blue, s));
  }

  /**
   * Exact dashboard-name match; an unknown or null name returns {@code fallback} (never throws), so
   * a stale saved setting cannot break boot.
   */
  public static LEDTheme fromDashboardName(String name, LEDTheme fallback) {
    if (name != null) {
      for (LEDTheme theme : values()) {
        if (theme.dashboardName.equals(name)) {
          return theme;
        }
      }
    }
    return fallback;
  }
}
