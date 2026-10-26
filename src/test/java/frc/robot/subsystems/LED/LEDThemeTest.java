package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.util.Color;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * The five Day of the Dead themes share one approved fusion design (user decision 2026-10-26):
 * alternating purple/orange bands (ABAB) at quarters of the path, so both colors appear across the
 * whole strip at any installed length. Stop colors are exact RGB design inputs, and the gradient
 * between adjacent stops is a per-channel RGB smoothstep — never an HSV hue-wheel interpolation,
 * which would detour through colors that are not in the design.
 */
class LEDThemeTest {
  // Stop colors are 8-bit design values; anything within one LSB is the same color.
  private static final double CHANNEL_EPS = 1.0 / 255.0 + 1e-9;
  private static final double EPS = 1e-9;

  // The unified fusion design: purple #7020D0 on the even quarter stops, orange #FF7900 on the odd.
  private static final int[] PURPLE = {112, 32, 208};
  private static final int[] ORANGE = {255, 121, 0};

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
    for (LEDTheme theme : LEDTheme.values()) {
      assertStop(theme, 0.00, PURPLE[0], PURPLE[1], PURPLE[2]);
      assertStop(theme, 0.25, ORANGE[0], ORANGE[1], ORANGE[2]);
      assertStop(theme, 0.50, PURPLE[0], PURPLE[1], PURPLE[2]);
      assertStop(theme, 0.75, ORANGE[0], ORANGE[1], ORANGE[2]);
      assertStop(theme, 1.00, PURPLE[0], PURPLE[1], PURPLE[2]);
    }
  }

  @Test
  void everyThemeRendersTheIdenticalFusionGradient() {
    // The five entries are color-identical presets (names + recommended effects differ only).
    for (double u = 0.0; u <= 1.0; u += 0.01) {
      Color reference = LEDTheme.MARIGOLD_BRIDGE.colorAt(u);
      for (LEDTheme theme : LEDTheme.values()) {
        assertColorEquals(reference, theme.colorAt(u), EPS, theme + " at u=" + u);
      }
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
    // The first segment runs u in [0, 0.25] from #7020D0 to #FF7900. At the segment midpoint
    // smoothstep(0.5) = 0.5, so every channel is the plain average of the two stop channels.
    Color mid = LEDTheme.FESTIVAL_NIGHT.colorAt(0.125);
    assertEquals((112.0 + 255.0) / 2.0 / 255.0, mid.red, CHANNEL_EPS, "midpoint red");
    assertEquals((32.0 + 121.0) / 2.0 / 255.0, mid.green, CHANNEL_EPS, "midpoint green");
    assertEquals((208.0 + 0.0) / 2.0 / 255.0, mid.blue, CHANNEL_EPS, "midpoint blue");
  }

  @Test
  void gradientStaysWithinTheTwoDesignColorsChannelEnvelope() {
    // Never the HSV hue wheel: per-channel RGB interpolation keeps every channel inside the
    // [min(purple, orange), max(purple, orange)] envelope of the two design colors, and each
    // channel is monotonic within every band (it only turns around exactly at the stops).
    for (int i = 0; i <= 100; i++) {
      double u = i * 0.01;
      Color c = LEDTheme.FESTIVAL_NIGHT.colorAt(u);
      String where = "at u=" + u;
      assertTrue(c.red >= 112.0 / 255.0 - CHANNEL_EPS, "red below envelope " + where);
      assertTrue(c.red <= 255.0 / 255.0 + CHANNEL_EPS, "red above envelope " + where);
      assertTrue(c.green >= 32.0 / 255.0 - CHANNEL_EPS, "green below envelope " + where);
      assertTrue(c.green <= 121.0 / 255.0 + CHANNEL_EPS, "green above envelope " + where);
      assertTrue(c.blue >= 0.0 - CHANNEL_EPS, "blue below envelope " + where);
      assertTrue(c.blue <= 208.0 / 255.0 + CHANNEL_EPS, "blue above envelope " + where);
    }
    // Halfway through the first band the color is a genuine RGB blend, not a hue-wheel detour
    // (which would drop red toward 0 while heading around through blue/cyan/green).
    Color halfBand = LEDTheme.FESTIVAL_NIGHT.colorAt(0.125);
    assertTrue(halfBand.red > 0.6, "mid-band red collapsed toward a wheel detour");
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
