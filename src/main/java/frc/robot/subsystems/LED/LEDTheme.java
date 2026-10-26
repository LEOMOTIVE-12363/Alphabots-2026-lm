package frc.robot.subsystems.LED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A Day of the Dead color theme: an ordered set of gradient stops over the logical ball-path
 * coordinate u (0 = intake entrance, 1 = shooter top). Stops are interpolated per RGB channel with
 * smoothstep easing — never around the HSV wheel — so the designed hues render exactly and no
 * unintended hues appear between stops.
 *
 * <p>Since 2026-10-26 (user decision) every theme renders the SAME orange + purple fusion gradient:
 * alternating purple/orange stripes with one color change roughly every {@link #FUSION_BAND_METERS}
 * of the configured strip, so both colors appear across the whole robot at any installed length —
 * never "purple head, orange tail". (The first cut used four broad ~1.75 m bands; the operator
 * asked for ~30 cm alternation the same day.) The five entries keep their names and recommended
 * idle effects as operator-facing presets; only the colors were unified. The stop RGB values are
 * the approved design inputs; keep them verbatim.
 */
public enum LEDTheme {
  MARIGOLD_BRIDGE("Marigold Bridge", IdleEffect.SLOW_FLOW, fusionStops()),
  FESTIVAL_NIGHT("Festival Night", IdleEffect.BREATHE, fusionStops()),
  PAPEL_FIESTA("Papel Fiesta", IdleEffect.CHASE, fusionStops()),
  ALEBRIJE_GLOW("Alebrije Glow", IdleEffect.SLOW_FLOW, fusionStops()),
  CANDLE_MEMORY("Candle Memory", IdleEffect.STATIC, fusionStops());

  /**
   * One fusion stripe length in meters (user decision 2026-10-26: "每 30 cm 就改一种颜色，大概"). The final
   * stripe before the strip end may be shorter when the configured length is not an exact multiple.
   */
  private static final double FUSION_BAND_METERS = 0.3;

  /**
   * The alternating fusion stop set over the configured strip: purple at u=0, then orange and
   * purple alternating every {@link #FUSION_BAND_METERS}, ending on purple at u=1. Stop positions
   * derive from {@link LEDConstants#LENGTH_METERS} so a future length edit keeps the physical
   * stripe width.
   */
  private static GradientStop[] fusionStops() {
    int bands = (int) Math.ceil(LEDConstants.LENGTH_METERS / FUSION_BAND_METERS);
    GradientStop[] stops = new GradientStop[bands + 1];
    for (int k = 0; k <= bands; k++) {
      double u = Math.min(k * FUSION_BAND_METERS / LEDConstants.LENGTH_METERS, 1.0);
      stops[k] = new GradientStop(u, k % 2 == 0 ? 0x7020D0 : 0xFF7900);
    }
    return stops;
  }

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
