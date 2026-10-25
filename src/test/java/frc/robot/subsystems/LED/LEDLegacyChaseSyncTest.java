package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * The base Pattern=Chase shares ONE lap between the external strip and the onboard thumbnail (third
 * acceptance round, 2026-10-25): the lap spans the configured external length plus the tail, both
 * renders advance through it with the same head position, and the thumbnail samples it at
 * normalized u positions like the theme thumbnail does. Before the fix the 8-px thumbnail ran its
 * own (8+6)/20 = 0.7 s cycle while the 200-px strip cycled in 10.3 s, so the two never stayed in
 * phase. These tests pin the shared lap across lengths, instants and wraparound, the single
 * brightness scaling, and that the Elastic preview (full-length render) follows the same lap.
 * Thumbnail u positions map onto the external strip's actual pixel span 0..REFERENCE-1 (fourth
 * acceptance round, 2026-10-26: u=1 previously pointed one pixel past the final external pixel).
 *
 * <p>The theme idle effect "Chase" (IdleEffect.CHASE) is a different animation and is not touched.
 */
class LEDLegacyChaseSyncTest {
  private static final int REFERENCE = LEDConstants.LENGTH_PIXELS;
  private static final int TAIL = LEDConstants.CHASE_TAIL_PIXELS;
  private static final double SPEED = LEDConstants.CHASE_SPEED_PIXELS_PER_SEC;
  private static final double LAP_SEC = (REFERENCE + TAIL) / SPEED;
  private static final int THUMBNAIL = LEDConstants.ONBOARD_THUMBNAIL_PIXELS;

  private static AddressableLEDBuffer render(int pixels, double now, double brightnessPercent) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(pixels);
    LED.renderLegacyInto(buffer, LEDPattern.CHASE, now, brightnessPercent);
    return buffer;
  }

  private static int packed(AddressableLEDBuffer buffer, int i) {
    return (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
  }

  private static int[] packedFrame(AddressableLEDBuffer buffer) {
    int[] values = new int[buffer.getLength()];
    for (int i = 0; i < values.length; i++) {
      values[i] = packed(buffer, i);
    }
    return values;
  }

  /** A reference HSV pixel built directly from the constants (independent of the render code). */
  private static int expectedPixel(double brightnessPercent, double distanceBehindHead) {
    int value = 0;
    if (distanceBehindHead >= 0 && distanceBehindHead < TAIL) {
      value = (int) (brightnessPercent / 100.0 * 255 * (1.0 - distanceBehindHead / TAIL));
    }
    AddressableLEDBuffer pixel = new AddressableLEDBuffer(1);
    pixel.setHSV(0, LEDConstants.CHASE_HUE, 255, value);
    return packed(pixel, 0);
  }

  @Test
  void thumbnailSharesTheExternalLapAndPhase() {
    // One external lap later the head returns to position zero on BOTH renders simultaneously:
    // strip pixel 0 and thumbnail pixel 0 (u=0) are at the head, bright and identical.
    for (double now : new double[] {0.0, LAP_SEC, 2 * LAP_SEC}) {
      AddressableLEDBuffer strip = render(REFERENCE, now, 100);
      AddressableLEDBuffer onboard = render(THUMBNAIL, now, 100);
      assertEquals(expectedPixel(100, 0), packed(strip, 0), "strip head at lap time " + now);
      assertEquals(expectedPixel(100, 0), packed(onboard, 0), "thumbnail head at lap time " + now);
    }

    // When the head reaches the thumbnail's far end (u=1 ⇒ position REFERENCE - 1, the external
    // strip's actual final pixel), thumbnail pixel 7 is the head. (Fourth acceptance round
    // 2026-10-26: u=1 previously mapped to the nonexistent pixel REFERENCE, one past the end.)
    double headAtFarEnd = (REFERENCE - 1) / SPEED;
    AddressableLEDBuffer onboard = render(THUMBNAIL, headAtFarEnd, 100);
    assertEquals(
        expectedPixel(100, 0), packed(onboard, THUMBNAIL - 1), "thumbnail far end at the head");
  }

  @Test
  void thumbnailPeriodIsTheExternalLapNotItsOwn() {
    int[] atZero = packedFrame(render(THUMBNAIL, 0.0, 100));
    // The old own-lap math had period (8+6)/20 = 0.7 s, so it repeated here; the shared lap must
    // not — at t=0.7 s the head sits at pixel position 14, between thumbnail samples (dark).
    int[] atOldLap = packedFrame(render(THUMBNAIL, (THUMBNAIL + TAIL) / SPEED, 100));
    assertNotEquals(atZero, atOldLap, "the thumbnail must not run its own short Chase cycle");

    // ...while the shared lap repeats exactly, on the thumbnail and the strip alike.
    assertArrayEquals(atZero, packedFrame(render(THUMBNAIL, LAP_SEC, 100)), "thumbnail lap");
    assertArrayEquals(
        packedFrame(render(REFERENCE, 0.0, 100)),
        packedFrame(render(REFERENCE, LAP_SEC, 100)),
        "strip lap (original external timing preserved)");
  }

  @Test
  void multipleThumbnailLengthsShareTheReferenceLap() {
    double now = 3.21; // arbitrary instant; head lands mid-lap
    double head = (now * SPEED) % (REFERENCE + TAIL);
    for (int length : new int[] {1, 4, THUMBNAIL, 16, 64, REFERENCE}) {
      AddressableLEDBuffer buffer = render(length, now, 100);
      for (int i = 0; i < length; i++) {
        // Thumbnail u spans the external strip's actual pixel endpoints 0..REFERENCE-1; a
        // single-pixel target sits at u=0 (position 0) by the length<=1 convention.
        double position = length == REFERENCE ? i : normalizedPosition(i, length) * (REFERENCE - 1);
        assertEquals(
            expectedPixel(100, head - position),
            packed(buffer, i),
            "length " + length + " pixel " + i + " must follow the shared reference lap");
      }
    }
  }

  private static double normalizedPosition(int i, int length) {
    return length <= 1 ? 0.0 : (double) i / (length - 1);
  }

  @Test
  void brightnessScalesTheSharedThumbnailExactlyOnce() {
    // At the head the value channel is the full brightness scale, so successive single scalings
    // are observable directly: 255 → 127 → 63; a second scaling would collapse 50% onto 63.
    double headAtFarEnd = (REFERENCE - 1) / SPEED; // head on the actual final pixel (u=1)
    assertEquals(
        expectedPixel(100, 0), packed(render(THUMBNAIL, headAtFarEnd, 100), THUMBNAIL - 1));
    assertEquals(expectedPixel(50, 0), packed(render(THUMBNAIL, headAtFarEnd, 50), THUMBNAIL - 1));
    assertEquals(expectedPixel(25, 0), packed(render(THUMBNAIL, headAtFarEnd, 25), THUMBNAIL - 1));
  }

  /** Minimal capture harness around the package-private LEDPreview constructor. */
  private static final class PreviewHarness implements AutoCloseable {
    final NetworkTableInstance nt = NetworkTableInstance.create();
    final Map<String, String[]> last = new LinkedHashMap<>();
    final LEDPreview preview =
        new LEDPreview(nt, () -> false, topic -> values -> last.put(topic, values.clone()));

    @Override
    public void close() {
      nt.close();
    }
  }

  @Test
  void previewChaseFollowsTheExternalLap() {
    try (PreviewHarness harness = new PreviewHarness()) {
      String topic = "/SmartDashboard/LED Preview/Current Selection";
      harness.preview.update(LEDPattern.CHASE, IdleEffect.STATIC, 0.0);
      String[] atZero = harness.last.get(topic).clone();
      harness.preview.update(LEDPattern.CHASE, IdleEffect.STATIC, 3.5);
      String[] midLap = harness.last.get(topic).clone();
      // At t=0 only the head (sample 0) is lit; at t=3.5 the head is mid-strip — a genuinely
      // different phase, so this sanity check distinguishes real content from an empty capture.
      org.junit.jupiter.api.Assertions.assertFalse(
          java.util.Arrays.equals(atZero, midLap), "the two preview instants must differ");

      harness.preview.update(LEDPattern.CHASE, IdleEffect.STATIC, LAP_SEC);
      String[] atLap = harness.last.get(topic).clone();
      assertArrayEquals(atZero, atLap, "the preview must repeat with the external lap");
    }
  }
}
