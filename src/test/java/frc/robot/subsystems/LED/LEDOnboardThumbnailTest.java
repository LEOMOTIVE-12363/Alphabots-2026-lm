package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LED.LEDSection.Role;
import java.util.function.Function;
import org.junit.jupiter.api.Test;

/**
 * The onboard thumbnail is a LOGICAL ball-path render, not a sampling of the physical strip: the
 * shared pipeline (theme × idle effect × brightness) is evaluated at eight normalized u positions
 * on a fixed non-reversed section, so onboard 0→7 is always intake→shooter regardless of how the
 * external strip is wired — reversed runs, split side panels and decoration sections can never
 * change the onboard direction or mix decoration into it. These tests pin that contract at the
 * render seam ({@link LED#renderThemeInto} / {@link LED#fillBlack}).
 */
class LEDOnboardThumbnailTest {
  private static final int THUMBNAIL_PIXELS = LEDConstants.ONBOARD_THUMBNAIL_PIXELS;
  private static final double NOW_SEC = 12.34;
  private static final IdleEffect EFFECT = IdleEffect.SLOW_FLOW;
  private static final Function<Double, Color> THEME =
      LEDPattern.MARIGOLD_BRIDGE.theme().orElseThrow()::colorAt;

  private static final LEDSection[] ONBOARD_LOGICAL = {
    new LEDSection(0, THUMBNAIL_PIXELS, false, Role.BALL_PATH)
  };

  /** Wiring variants the external strip might eventually be configured as (16 px stand-in). */
  private static LEDSection[][] wiringVariants() {
    return new LEDSection[][] {
      {new LEDSection(0, 16, false, Role.BALL_PATH)}, // one forward run
      {new LEDSection(0, 16, true, Role.BALL_PATH)}, // reversed run (intake at the far end)
      { // two concatenated side panels, the second physically mirrored
        new LEDSection(0, 8, false, Role.BALL_PATH), new LEDSection(8, 8, true, Role.BALL_PATH)
      },
      { // decoration run in front of the ball path
        new LEDSection(0, 4, false, Role.DECORATION), new LEDSection(4, 12, false, Role.BALL_PATH)
      },
      { // ball path with a decoration back panel behind it
        new LEDSection(0, 12, false, Role.BALL_PATH), new LEDSection(12, 4, false, Role.DECORATION)
      }
    };
  }

  private static AddressableLEDBuffer renderThumbnail() {
    AddressableLEDBuffer onboard = new AddressableLEDBuffer(THUMBNAIL_PIXELS);
    LED.renderThemeInto(onboard, ONBOARD_LOGICAL, THEME, EFFECT, NOW_SEC, 1.0);
    return onboard;
  }

  @Test
  void onboardMathIsTheSharedPipelineAtNormalizedPositions() {
    AddressableLEDBuffer onboard = renderThumbnail();

    for (int k = 0; k < THUMBNAIL_PIXELS; k++) {
      double u = THUMBNAIL_PIXELS <= 1 ? 0.0 : (double) k / (THUMBNAIL_PIXELS - 1);
      Color expected =
          IdleEffects.scale(
              IdleEffects.colorAt(EFFECT, THEME, u, NOW_SEC),
              IdleEffects.envelope(EFFECT, u, NOW_SEC));
      assertEquals(IdleEffects.to255(expected.red), onboard.getRed(k), "red at u=" + u);
      assertEquals(IdleEffects.to255(expected.green), onboard.getGreen(k), "green at u=" + u);
      assertEquals(IdleEffects.to255(expected.blue), onboard.getBlue(k), "blue at u=" + u);
    }
  }

  @Test
  void onboardIsIdenticalAcrossExternalWiringVariants() {
    AddressableLEDBuffer reference = renderThumbnail();
    assertNotEquals(
        new AddressableLEDBuffer(THUMBNAIL_PIXELS).getRed(0),
        reference.getRed(0),
        "sanity: the reference render actually shows something");

    AddressableLEDBuffer firstExternal = null;
    LEDSection[][] variants = wiringVariants();
    for (int variantIndex = 0; variantIndex < variants.length; variantIndex++) {
      LEDSection[] wiring = variants[variantIndex];
      AddressableLEDBuffer external = new AddressableLEDBuffer(16);
      LED.renderThemeInto(external, wiring, THEME, EFFECT, NOW_SEC, 1.0);
      AddressableLEDBuffer onboard = renderThumbnail();

      assertArrayEquals(
          packed(reference), packed(onboard), "onboard must not depend on external wiring");
      if (firstExternal == null) {
        firstExternal = external;
      } else if (variantIndex == 1) {
        // Sanity (the variants really do differ): a reversed ball path mirrors the physical strip.
        org.junit.jupiter.api.Assertions.assertFalse(
            java.util.Arrays.equals(packed(firstExternal), packed(external)),
            "sanity: reversed wiring mirrors the strip");
      }
    }
  }

  @Test
  void offAndZeroBrightnessClearTheThumbnail() {
    AddressableLEDBuffer zeroed = renderThumbnail();
    AddressableLEDBuffer dark = new AddressableLEDBuffer(THUMBNAIL_PIXELS);
    LED.renderThemeInto(dark, ONBOARD_LOGICAL, THEME, EFFECT, NOW_SEC, 0.0);
    for (int k = 0; k < THUMBNAIL_PIXELS; k++) {
      assertEquals(0, dark.getRed(k), "brightness 0 must dark the thumbnail");
      assertEquals(0, dark.getGreen(k), "brightness 0 must dark the thumbnail");
      assertEquals(0, dark.getBlue(k), "brightness 0 must dark the thumbnail");
    }

    // Off fills the thumbnail black through the same path periodic uses.
    AddressableLEDBuffer off = new AddressableLEDBuffer(THUMBNAIL_PIXELS);
    for (int k = 0; k < THUMBNAIL_PIXELS; k++) {
      off.setRGB(k, 200, 30, 30); // lit sentinel: Off must clear it, not skip it
    }
    LED.fillBlack(off);
    for (int k = 0; k < THUMBNAIL_PIXELS; k++) {
      assertEquals(0, off.getRed(k), "Off must clear the thumbnail");
      assertEquals(0, off.getGreen(k), "Off must clear the thumbnail");
      assertEquals(0, off.getBlue(k), "Off must clear the thumbnail");
    }
    assertNotEquals(0, zeroed.getRed(0), "sanity: 100% render was not dark");
  }

  @Test
  void brightnessIsScaledExactlyOnceOnTheThumbnail() {
    AddressableLEDBuffer full = renderThumbnail();
    AddressableLEDBuffer half = new AddressableLEDBuffer(THUMBNAIL_PIXELS);
    LED.renderThemeInto(half, ONBOARD_LOGICAL, THEME, EFFECT, NOW_SEC, 0.5);

    for (int k = 0; k < THUMBNAIL_PIXELS; k++) {
      Color expected = IdleEffects.scale(full.getLED(k), 0.5);
      // Double scaling would land near 0.25×, far from the 0.5× the render layer applied once.
      assertEquals(IdleEffects.to255(expected.red), half.getRed(k), "red at " + k);
      assertEquals(IdleEffects.to255(expected.green), half.getGreen(k), "green at " + k);
      assertEquals(IdleEffects.to255(expected.blue), half.getBlue(k), "blue at " + k);
    }
  }

  private static int[] packed(AddressableLEDBuffer buffer) {
    int[] values = new int[buffer.getLength()];
    for (int i = 0; i < values.length; i++) {
      values[i] = (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
    }
    return values;
  }
}
