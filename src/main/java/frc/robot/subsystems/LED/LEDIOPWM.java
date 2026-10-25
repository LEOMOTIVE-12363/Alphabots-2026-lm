package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * roboRIO PWM output driving the WS2811 COB strip (green data wire → PWM signal pin, strip power
 * from the PDP, grounds tied). Also used in sim, where AddressableLED maps to a harmless HAL sim
 * device.
 *
 * <p>Color order (third acceptance round, 2026-10-25): the one physical contract shared with the
 * CANdle driver is that {@code swapRedGreen == true} ⇔ the installed strip orders its channels GRB
 * (classic WS2811). The WPILib 2026.2.1 HAL serializes each buffer pixel in the configured {@link
 * AddressableLED.ColorOrder} — verified against the shipped HAL sources: the FPGA natively outputs
 * GRB, the default color order is GRB (athena/AddressableLED.cpp), and any other order is converted
 * exactly once before the data reaches the FPGA. This driver therefore sets the HAL color order
 * explicitly (never relying on the default) and copies logical RGB through unchanged, mirroring the
 * CANdle driver's StripType mapping — switching {@code LEDConstants.HARDWARE} alone can never
 * change the wire channel order. Brightness is already applied by the render layer and is not
 * touched here.
 */
public class LEDIOPWM implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer outputBuffer;

  public LEDIOPWM(int pwmPort, int lengthPixels, boolean swapRedGreen) {
    led = new AddressableLED(pwmPort);
    led.setLength(lengthPixels);
    led.setColorOrder(stripColorOrder(swapRedGreen));
    outputBuffer = new AddressableLEDBuffer(lengthPixels);
    led.setData(outputBuffer); // start dark
    led.start();
  }

  /**
   * The HAL color order matching the strip's physical channel order under the shared contract: GRB
   * when the WS2811 R/G swap flag is set, RGB otherwise.
   */
  static AddressableLED.ColorOrder stripColorOrder(boolean swapRedGreen) {
    return swapRedGreen ? AddressableLED.ColorOrder.kGRB : AddressableLED.ColorOrder.kRGB;
  }

  @Override
  public void writeBuffer(AddressableLEDBuffer buffer) {
    copyLogicalRgb(buffer, outputBuffer);
    led.setData(outputBuffer);
  }

  /**
   * Copies logical-RGB colors unchanged (bounded by the shorter buffer): the physical channel-order
   * conversion happens exactly once, in the HAL, per the configured color order.
   */
  static void copyLogicalRgb(AddressableLEDBuffer source, AddressableLEDBuffer destination) {
    int length = Math.min(source.getLength(), destination.getLength());
    for (int i = 0; i < length; i++) {
      destination.setRGB(i, source.getRed(i), source.getGreen(i), source.getBlue(i));
    }
  }
}
