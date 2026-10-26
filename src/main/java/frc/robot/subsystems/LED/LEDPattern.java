package frc.robot.subsystems.LED;

import java.util.Optional;

/**
 * Built-in styles for the LED strip, selectable from the Elastic "LED Pattern" chooser.
 *
 * <p>Since 2026-10-26 the menu is exactly {@code Off} plus the three Day of the Dead themes (user
 * decision): the former legacy self-colored patterns (Solid, Rainbow, Chase, Breathe) and the
 * removed themes (Papel Fiesta, Candle Memory) are gone. Each theme pattern renders its {@link
 * LEDTheme} gradient with the independently chosen {@link IdleEffect} — Off is the only non-theme
 * pattern and renders black.
 */
public enum LEDPattern {
  OFF("Off"),
  MARIGOLD_BRIDGE(LEDTheme.MARIGOLD_BRIDGE),
  FESTIVAL_NIGHT(LEDTheme.FESTIVAL_NIGHT),
  ALEBRIJE_GLOW(LEDTheme.ALEBRIJE_GLOW);

  private final String dashboardName;
  private final LEDTheme theme; // null for Off

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

  /** The theme this pattern renders, empty for {@link #OFF}. */
  public Optional<LEDTheme> theme() {
    return Optional.ofNullable(theme);
  }

  /**
   * Exact dashboard-name match; an unknown or null name returns {@code fallback} (never throws).
   * This is the boot-restore migration seam: a saved name from an older build — e.g. the removed
   * legacy patterns or themes — resolves to the fallback instead of breaking boot.
   */
  public static LEDPattern fromDashboardName(String name, LEDPattern fallback) {
    if (name != null) {
      for (LEDPattern pattern : values()) {
        if (pattern.dashboardName.equals(name)) {
          return pattern;
        }
      }
    }
    return fallback;
  }
}
