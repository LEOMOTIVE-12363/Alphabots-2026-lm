package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.junit.jupiter.api.Test;

/**
 * The PWM driver hands logical RGB to the HAL unchanged and configures the HAL color order from the
 * shared strip contract (true = GRB strip). The conversion happens exactly once, inside the HAL
 * (verified against the WPILib 2026.2.1 HAL sources: the FPGA natively outputs GRB and any other
 * configured order is converted once before the data reaches it). These tests pin the copy and the
 * flag→HAL-order mapping; the constructor's setColorOrder call itself runs in the PWM simulation
 * smoke — constructing AddressableLED in unit tests is avoided on purpose because the simulated
 * HAL's PWM port allocation is JVM-global and never released (see AGENTS).
 *
 * <p>Before the 2026-10-25 third acceptance round the driver software-swapped R/G when the flag was
 * set while leaving the HAL at its GRB default, which meant the opposite physical order from the
 * CANdle driver at the same flag value; the cross-driver contract lives in {@link
 * LEDColorOrderContractTest}.
 */
class LEDIOPWMTest {

  @Test
  void copyPreservesLogicalRgbForEitherStripOrder() {
    // The buffer stays logical RGB whichever order the strip uses — the HAL does the one
    // conversion, so a software swap here would be the second conversion.
    for (boolean stripOrderFlag : new boolean[] {false, true}) {
      AddressableLEDBuffer source = filled(5);
      AddressableLEDBuffer destination = new AddressableLEDBuffer(5);

      LEDIOPWM.copyLogicalRgb(source, destination);

      for (int i = 0; i < 5; i++) {
        assertEquals(source.getRed(i), destination.getRed(i), "red at " + i);
        assertEquals(source.getGreen(i), destination.getGreen(i), "green at " + i);
        assertEquals(source.getBlue(i), destination.getBlue(i), "blue at " + i);
      }
    }
  }

  @Test
  void flagMapsToTheMatchingHalColorOrder() {
    assertEquals(AddressableLED.ColorOrder.kGRB, LEDIOPWM.stripColorOrder(true), "GRB strip");
    assertEquals(AddressableLED.ColorOrder.kRGB, LEDIOPWM.stripColorOrder(false), "RGB strip");
  }

  @Test
  void copyHandlesShorterDestinationBuffers() {
    AddressableLEDBuffer source = filled(8);
    AddressableLEDBuffer destination = new AddressableLEDBuffer(3);

    // Must not throw or read/write past either buffer.
    LEDIOPWM.copyLogicalRgb(source, destination);

    assertEquals(source.getRed(2), destination.getRed(2));
    assertEquals(source.getGreen(2), destination.getGreen(2));
    assertEquals(source.getBlue(2), destination.getBlue(2));
  }

  private static AddressableLEDBuffer filled(int pixels) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(pixels);
    for (int i = 0; i < pixels; i++) {
      buffer.setRGB(i, 10 + i, 20 + i, 30 + i);
    }
    return buffer;
  }
}
