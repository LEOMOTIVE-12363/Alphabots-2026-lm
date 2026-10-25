package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The two drivers implement ONE physical color-order contract: {@code
 * LEDConstants.SWAP_RED_GREEN_FOR_WS2811} describes the strip's physical channel order (true = GRB,
 * classic WS2811; false = RGB). Verified platform facts this regression connects:
 *
 * <ul>
 *   <li>PWM (WPILib 2026.2.1 HAL, checked against the shipped sources): the HAL serializes each
 *       buffer pixel in the configured {@link AddressableLED.ColorOrder} — the FPGA natively
 *       outputs GRB and any other order is converted exactly once (athena/AddressableLED.cpp; the
 *       default order is GRB, which is why the driver must set it explicitly instead of relying on
 *       the default as it did before the 2026-10-25 third acceptance round).
 *   <li>CANdle (Phoenix 6): {@code StripType} tells the firmware the attached strip's channel
 *       order; paint payloads are logical RGB and the firmware serializes per StripType (the
 *       onboard LEDs are firmware-compensated, so they are beyond this contract).
 * </ul>
 *
 * <p>Therefore the wire channel sequence a driver produces is: reorder the logical RGB channels by
 * the configured order. This test derives that sequence for BOTH drivers through their real entry
 * points — the PWM driver's HAL order + output buffer, and the CANdle driver's recorded strip-type
 * config + paint payloads — and asserts they agree for pure red, green and blue at both flag
 * values. Under the pre-fix code the two drivers disagreed at every flag value (pure red wired its
 * 255 byte into opposite positions).
 */
class LEDColorOrderContractTest {

  /** Records the CANdle-side device requests; a pure-color frame drives the payload check. */
  private static final class RecordingTransport implements LEDIOCandle.Transport {
    StripTypeValue stripType;
    final List<int[]> paints = new ArrayList<>();

    @Override
    public StatusCode configureStrip(StripTypeValue stripType) {
      this.stripType = stripType;
      return StatusCode.OK;
    }

    @Override
    public StatusCode clearAnimations() {
      return StatusCode.OK;
    }

    @Override
    public StatusCode paint(int start, int end, int red, int green, int blue) {
      paints.add(new int[] {start, end, red, green, blue});
      return StatusCode.OK;
    }
  }

  static Stream<Object[]> flagAndPureColors() {
    return Stream.of(
        new Object[] {false, 255, 0, 0},
        new Object[] {false, 0, 255, 0},
        new Object[] {false, 0, 0, 255},
        new Object[] {true, 255, 0, 0},
        new Object[] {true, 0, 255, 0},
        new Object[] {true, 0, 0, 255});
  }

  @ParameterizedTest(name = "flag={0}, color=({1},{2},{3})")
  @MethodSource("flagAndPureColors")
  void bothDriversWireTheSameTrueChannels(boolean stripIsGrb, int red, int green, int blue) {
    // ---- PWM side: the configured HAL order and the buffer handed to setData.
    AddressableLED.ColorOrder halOrder = LEDIOPWM.stripColorOrder(stripIsGrb);
    AddressableLEDBuffer source = new AddressableLEDBuffer(8);
    for (int i = 0; i < 8; i++) {
      source.setRGB(i, red, green, blue);
    }
    AddressableLEDBuffer pwmOutput = new AddressableLEDBuffer(8);
    LEDIOPWM.copyLogicalRgb(source, pwmOutput); // identity: the one conversion is the HAL's
    String pwmOrderName = halOrder.name().substring(1); // kGRB -> "GRB", kRGB -> "RGB"

    // ---- CANdle side: the recorded strip-type config and paint payloads.
    RecordingTransport transport = new RecordingTransport();
    LEDIOCandle candle = new LEDIOCandle(transport, 8, stripIsGrb, 0.050, 64, () -> 0.0);
    AddressableLEDBuffer external = new AddressableLEDBuffer(8);
    for (int i = 0; i < 8; i++) {
      external.setRGB(i, red, green, blue);
    }
    candle.writeOnboardBuffer(external);
    candle.writeBuffer(external);
    String candleOrderName = transport.stripType.name(); // "GRB" / "RGB"

    // Same configured physical order, logical payloads everywhere (paint 0 is the constructor's
    // onboard dimming, so the payload check starts after it)...
    assertEquals(pwmOrderName, candleOrderName, "both drivers must configure the same order");
    assertNotEquals(
        LEDIOPWM.stripColorOrder(true), LEDIOPWM.stripColorOrder(false), "orders must differ");
    assertTrue(transport.paints.size() > 1, "the pure-color frame must have been painted");
    for (int[] paint : transport.paints.subList(1, transport.paints.size())) {
      assertEquals(red, paint[2], "CANdle payload must stay logical red");
      assertEquals(green, paint[3], "CANdle payload must stay logical green");
      assertEquals(blue, paint[4], "CANdle payload must stay logical blue");
    }
    assertEquals(red, pwmOutput.getRed(3), "PWM buffer must stay logical red");
    assertEquals(green, pwmOutput.getGreen(3), "PWM buffer must stay logical green");
    assertEquals(blue, pwmOutput.getBlue(3), "PWM buffer must stay logical blue");

    // ...therefore identical wire channel sequences: the strip's like-named channel receives the
    // like-named logical value on both platforms.
    assertArrayEquals(
        wireBytes(pwmOrderName, pwmOutput.getRed(3), pwmOutput.getGreen(3), pwmOutput.getBlue(3)),
        wireBytes(candleOrderName, red, green, blue),
        "the two drivers must wire the same physical channel order for the same flag");
  }

  /**
   * The wire byte sequence a platform emits for a logical color under a channel order "GRB"/"RGB".
   */
  private static int[] wireBytes(String order, int red, int green, int blue) {
    int[] channels = {red, green, blue};
    int[] wire = new int[3];
    for (int i = 0; i < 3; i++) {
      wire[i] = channels["RGB".indexOf(order.charAt(i))];
    }
    return wire;
  }
}
