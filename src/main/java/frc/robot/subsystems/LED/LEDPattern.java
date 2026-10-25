package frc.robot.subsystems.LED;

import java.util.Optional;

/**
 * Built-in effects for the LED strip, selectable from the Elastic "LED Pattern" chooser.
 *
 * <p>The first five are the legacy self-colored effects rendered directly by {@link LED}. The five
 * Day of the Dead options each bind a {@link LEDTheme}: they render the theme gradient with the
 * independently chosen {@link IdleEffect}.
 */
public enum LEDPattern {
  OFF("Off"),
  SOLID("Solid"),
  RAINBOW("Rainbow"),
  CHASE("Chase"),
  BREATHE("Breathe"),
  MARIGOLD_BRIDGE(LEDTheme.MARIGOLD_BRIDGE),
  FESTIVAL_NIGHT(LEDTheme.FESTIVAL_NIGHT),
  PAPEL_FIESTA(LEDTheme.PAPEL_FIESTA),
  ALEBRIJE_GLOW(LEDTheme.ALEBRIJE_GLOW),
  CANDLE_MEMORY(LEDTheme.CANDLE_MEMORY);

  private final String dashboardName;
  private final LEDTheme theme; // null for the five legacy patterns

  LEDPattern(String dashboardName) {
    this.dashboardName = dashboardName;
    this.theme = null;
  }

  LEDPattern(LEDTheme theme) {
    this.dashboardName = theme.getDashboardName();
    this.theme = theme;
  }

  /** Label shown for this pattern on the dashboard chooser. */
  public String getDashboardName() {
    return dashboardName;
  }

  /** The theme this pattern renders, empty for the five legacy patterns. */
  public Optional<LEDTheme> theme() {
    return Optional.ofNullable(theme);
  }
}
