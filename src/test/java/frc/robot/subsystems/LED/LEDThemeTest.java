package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.util.Color;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * The three Day of the Dead themes are the approved visual design: their stop colors are exact RGB
 * design inputs, and the gradient between adjacent stops is a per-channel RGB smoothstep — never an
 * HSV hue-wheel interpolation, which would detour through colors that are not in the design (purple
 * to orange around the wheel passes through blue and green).
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

    // Orange + purple (user requirement): the former rose middle stop #FF167A was removed on
    // 2026-10-26, leaving a clean two-color gradient.
    assertStop(LEDTheme.FESTIVAL_NIGHT, 0.0, 112, 32, 208);
    assertStop(LEDTheme.FESTIVAL_NIGHT, 1.0, 255, 121, 0);

    assertStop(LEDTheme.ALEBRIJE_GLOW, 0.0, 86, 54, 255);
    assertStop(LEDTheme.ALEBRIJE_GLOW, 0.60, 0, 214, 178);
    assertStop(LEDTheme.ALEBRIJE_GLOW, 1.0, 255, 36, 140);
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
    // Festival Night is a single [0, 1] segment from #7020D0 to #FF7900. At the midpoint
    // smoothstep(0.5) = 0.5, so every channel is the plain average of the two stop channels.
    Color mid = LEDTheme.FESTIVAL_NIGHT.colorAt(0.5);
    assertEquals((112.0 + 255.0) / 2.0 / 255.0, mid.red, CHANNEL_EPS, "midpoint red");
    assertEquals((32.0 + 121.0) / 2.0 / 255.0, mid.green, CHANNEL_EPS, "midpoint green");
    assertEquals((208.0 + 0.0) / 2.0 / 255.0, mid.blue, CHANNEL_EPS, "midpoint blue");
  }

  @Test
  void gradientNeverTakesTheHueWheelDetour() {
    // #7020D0 (purple) to #FF7900 (orange): one way around the HSV wheel passes through blue,
    // cyan, and green, where the red channel falls to ~0. Per-channel RGB interpolation keeps red
    // climbing monotonically between the two stop channels instead.
    double prevRed = -1.0;
    for (int i = 0; i <= 100; i++) {
      double u = i * 0.01;
      double red = LEDTheme.FESTIVAL_NIGHT.colorAt(u).red;
      assertTrue(red >= 112.0 / 255.0 - CHANNEL_EPS, "red fell below the start stop at u=" + u);
      assertTrue(red >= prevRed - EPS, "red must climb monotonically, dipped at u=" + u);
      prevRed = red;
    }
    // Late in the gradient the color is essentially the orange stop, not a hue-wheel halfway color.
    assertTrue(LEDTheme.FESTIVAL_NIGHT.colorAt(0.8).red > 0.9);
  }

  @Test
  void dashboardNamesMatchTheApprovedThemeNamesAndAreUnique() {
    assertEquals("Marigold Bridge", LEDTheme.MARIGOLD_BRIDGE.getDashboardName());
    assertEquals("Festival Night", LEDTheme.FESTIVAL_NIGHT.getDashboardName());
    assertEquals("Alebrije Glow", LEDTheme.ALEBRIJE_GLOW.getDashboardName());

    Set<String> names = new HashSet<>();
    for (LEDTheme theme : LEDTheme.values()) {
      assertTrue(names.add(theme.getDashboardName()), "duplicate dashboard name: " + theme);
    }
    assertEquals(3, names.size(), "the menu carries exactly three themes");
  }

  @Test
  void recommendedEffectsMatchTheDesign() {
    assertSame(IdleEffect.SLOW_FLOW, LEDTheme.MARIGOLD_BRIDGE.getRecommendedEffect());
    assertSame(IdleEffect.BREATHE, LEDTheme.FESTIVAL_NIGHT.getRecommendedEffect());
    assertSame(IdleEffect.SLOW_FLOW, LEDTheme.ALEBRIJE_GLOW.getRecommendedEffect());
  }

  @Test
  void fromDashboardNameResolvesExactNamesAndFallsBackOtherwise() {
    for (LEDTheme theme : LEDTheme.values()) {
      assertSame(theme, LEDTheme.fromDashboardName(theme.getDashboardName(), null));
    }
    // Removed themes must resolve to the fallback — they cannot break boot or resurrect.
    for (String removed : new String[] {"Papel Fiesta", "Candle Memory"}) {
      assertSame(
          LEDTheme.MARIGOLD_BRIDGE,
          LEDTheme.fromDashboardName(removed, LEDTheme.MARIGOLD_BRIDGE),
          "removed theme " + removed + " must fall back");
    }
    assertSame(
        LEDTheme.MARIGOLD_BRIDGE,
        LEDTheme.fromDashboardName("Not A Theme", LEDTheme.MARIGOLD_BRIDGE));
    assertSame(
        LEDTheme.MARIGOLD_BRIDGE, LEDTheme.fromDashboardName(null, LEDTheme.MARIGOLD_BRIDGE));
    // Exact match only: a case variant of a real name is an unknown name.
    assertSame(
        LEDTheme.MARIGOLD_BRIDGE,
        LEDTheme.fromDashboardName("marigold bridge", LEDTheme.MARIGOLD_BRIDGE));
  }
}
