package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.util.Color;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * The five Day of the Dead themes have distinct approved palettes. Their stop colors are exact RGB
 * design inputs, and the gradient between adjacent stops is a per-channel RGB smoothstep — never an
 * HSV hue-wheel interpolation, which would detour through colors that are not in the design.
 */
class LEDThemeTest {
  // Stop colors are 8-bit design values; anything within one LSB is the same color.
  private static final double CHANNEL_EPS = 1.0 / 255.0 + 1e-9;
  private static final double EPS = 1e-9;

  private static void assertStop(LEDTheme theme, double u, int r, int g, int b) {
    Color c = theme.colorAt(u);
    String where = theme.getDashboardName() + " @ u=" + u;
    assertEquals(r / 255.0, c.red, CHANNEL_EPS, where + " red");
    assertEquals(g / 255.0, c.green, CHANNEL_EPS, where + " green");
    assertEquals(b / 255.0, c.blue, CHANNEL_EPS, where + " blue");
  }

  private static void assertColorEquals(Color expected, Color actual, double eps, String where) {
    assertEquals(expected.red, actual.red, eps, where + " red");
    assertEquals(expected.green, actual.green, eps, where + " green");
    assertEquals(expected.blue, actual.blue, eps, where + " blue");
  }

  @Test
  void everyStopReproducesItsExactDesignColor() {
    assertStop(LEDTheme.MARIGOLD_BRIDGE, 0.0, 255, 101, 0);
    assertStop(LEDTheme.MARIGOLD_BRIDGE, 0.62, 255, 188, 36);
    assertStop(LEDTheme.MARIGOLD_BRIDGE, 1.0, 255, 149, 0);

    assertStop(LEDTheme.FESTIVAL_NIGHT, 0.0, 112, 32, 208);
    assertStop(LEDTheme.FESTIVAL_NIGHT, 0.70, 255, 22, 122);
    assertStop(LEDTheme.FESTIVAL_NIGHT, 1.0, 255, 121, 0);

    assertStop(LEDTheme.PAPEL_FIESTA, 0.0, 255, 121, 0);
    assertStop(LEDTheme.PAPEL_FIESTA, 0.33, 255, 22, 122);
    assertStop(LEDTheme.PAPEL_FIESTA, 0.67, 0, 201, 173);
    assertStop(LEDTheme.PAPEL_FIESTA, 1.0, 255, 208, 64);

    assertStop(LEDTheme.ALEBRIJE_GLOW, 0.0, 86, 54, 255);
    assertStop(LEDTheme.ALEBRIJE_GLOW, 0.60, 0, 214, 178);
    assertStop(LEDTheme.ALEBRIJE_GLOW, 1.0, 255, 36, 140);

    assertStop(LEDTheme.CANDLE_MEMORY, 0.0, 255, 149, 0);
    assertStop(LEDTheme.CANDLE_MEMORY, 1.0, 255, 193, 90);
  }

  @Test
  void themesHaveDistinctColorPalettes() {
    Set<String> palettes = new HashSet<>();
    for (LEDTheme theme : LEDTheme.values()) {
      StringBuilder palette = new StringBuilder();
      for (double u : new double[] {0.0, 0.33, 0.60, 0.67, 1.0}) {
        Color c = theme.colorAt(u);
        palette
            .append(Math.round(c.red * 255))
            .append(',')
            .append(Math.round(c.green * 255))
            .append(',')
            .append(Math.round(c.blue * 255))
            .append(';');
      }
      assertTrue(
          palettes.add(palette.toString()),
          theme + " duplicates another theme palette: " + palette);
    }
  }

  @Test
  void outOfRangeUClampsToTheEndpointStops() {
    for (LEDTheme theme : LEDTheme.values()) {
      assertColorEquals(theme.colorAt(0.0), theme.colorAt(-0.4), EPS, theme + " below range");
      assertColorEquals(theme.colorAt(1.0), theme.colorAt(1.6), EPS, theme + " above range");
    }
  }

  @Test
  void segmentMidpointIsAPerChannelRgbSmoothstep() {
    // Festival Night's first segment runs u in [0, 0.70] from #7020D0 to #FF167A. At the midpoint
    // smoothstep(0.5) = 0.5, so every channel is the plain average of the two stop channels.
    Color mid = LEDTheme.FESTIVAL_NIGHT.colorAt(0.35);
    assertEquals((112.0 + 255.0) / 2.0 / 255.0, mid.red, CHANNEL_EPS, "midpoint red");
    assertEquals((32.0 + 22.0) / 2.0 / 255.0, mid.green, CHANNEL_EPS, "midpoint green");
    assertEquals((208.0 + 122.0) / 2.0 / 255.0, mid.blue, CHANNEL_EPS, "midpoint blue");
  }

  @Test
  void gradientNeverTakesTheHueWheelDetour() {
    // #7020D0 (purple) to #FF167A (pink): one way around the HSV wheel passes through blue, cyan,
    // and green, where the red channel falls to ~0. Per-channel RGB interpolation keeps red
    // climbing monotonically between the two stop channels instead.
    double prevRed = -1.0;
    for (int i = 0; i <= 70; i++) {
      double u = i * 0.01;
      double red = LEDTheme.FESTIVAL_NIGHT.colorAt(u).red;
      assertTrue(red >= 112.0 / 255.0 - CHANNEL_EPS, "red fell below the start stop at u=" + u);
      assertTrue(red >= prevRed - EPS, "red must climb monotonically, dipped at u=" + u);
      prevRed = red;
    }
    // Late in the segment the color is essentially the pink stop, not a hue-wheel halfway color.
    assertTrue(LEDTheme.FESTIVAL_NIGHT.colorAt(0.63).red > 0.9);
  }

  @Test
  void dashboardNamesMatchTheApprovedThemeNamesAndAreUnique() {
    assertEquals("Marigold Bridge", LEDTheme.MARIGOLD_BRIDGE.getDashboardName());
    assertEquals("Festival Night", LEDTheme.FESTIVAL_NIGHT.getDashboardName());
    assertEquals("Papel Fiesta", LEDTheme.PAPEL_FIESTA.getDashboardName());
    assertEquals("Alebrije Glow", LEDTheme.ALEBRIJE_GLOW.getDashboardName());
    assertEquals("Candle Memory", LEDTheme.CANDLE_MEMORY.getDashboardName());

    Set<String> names = new HashSet<>();
    for (LEDTheme theme : LEDTheme.values()) {
      assertTrue(names.add(theme.getDashboardName()), "duplicate dashboard name: " + theme);
    }
  }

  @Test
  void recommendedEffectsMatchTheDesign() {
    assertSame(IdleEffect.SLOW_FLOW, LEDTheme.MARIGOLD_BRIDGE.getRecommendedEffect());
    assertSame(IdleEffect.BREATHE, LEDTheme.FESTIVAL_NIGHT.getRecommendedEffect());
    assertSame(IdleEffect.CHASE, LEDTheme.PAPEL_FIESTA.getRecommendedEffect());
    assertSame(IdleEffect.SLOW_FLOW, LEDTheme.ALEBRIJE_GLOW.getRecommendedEffect());
    assertSame(IdleEffect.STATIC, LEDTheme.CANDLE_MEMORY.getRecommendedEffect());
  }

  @Test
  void fromDashboardNameResolvesExactNamesAndFallsBackOtherwise() {
    for (LEDTheme theme : LEDTheme.values()) {
      assertSame(theme, LEDTheme.fromDashboardName(theme.getDashboardName(), null));
    }
    assertSame(
        LEDTheme.CANDLE_MEMORY, LEDTheme.fromDashboardName("Not A Theme", LEDTheme.CANDLE_MEMORY));
    assertSame(LEDTheme.CANDLE_MEMORY, LEDTheme.fromDashboardName(null, LEDTheme.CANDLE_MEMORY));
    // Exact match only: a case variant of a real name is an unknown name.
    assertSame(
        LEDTheme.CANDLE_MEMORY,
        LEDTheme.fromDashboardName("marigold bridge", LEDTheme.CANDLE_MEMORY));
  }
}
