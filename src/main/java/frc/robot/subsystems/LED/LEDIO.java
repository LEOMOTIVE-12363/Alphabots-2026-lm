package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

/**
 * Hardware abstraction for the addressable LED strip. Buffers handed to {@link #writeBuffer} hold
 * logical RGB with global brightness already applied by the render layer; each implementation
 * configures its platform's physical channel order from the shared {@code
 * LEDConstants.SWAP_RED_GREEN_FOR_WS2811} contract (true = GRB strip) so the conversion happens
 * exactly once in hardware — the PWM driver sets the HAL color order, the CANdle driver sets the
 * strip type — and must not scale brightness again.
 *
 * <p>The empty implementation is used in replay so logged runs stay deterministic (no hardware
 * output, no NetworkTables, no settings-file writes).
 */
public interface LEDIO {
  /** Pushes the latest external-strip buffer to the hardware. */
  default void writeBuffer(AddressableLEDBuffer buffer) {}

  /**
   * Pushes the onboard-thumbnail buffer (CANdle device indices 0-7): logical ball-path colors,
   * brightness already applied by the render layer, logical 0 = intake end. Only the CANdle driver
   * consumes it; PWM and replay keep the no-op default. Call before {@link #writeBuffer} in the
   * same cycle so a driver planning both segments at once sees current thumbnail colors.
   */
  default void writeOnboardBuffer(AddressableLEDBuffer buffer) {}

  /**
   * The single driver-selection entry: constructs only the driver chosen by {@link
   * LEDConstants#HARDWARE} in REAL/SIM (the other physical device is never opened), and the empty
   * implementation in REPLAY. Switching hardware means editing that one constant and redeploying.
   */
  static LEDIO create() {
    return create(Constants.currentMode, LEDConstants.HARDWARE);
  }

  /** Injectable sibling of {@link #create()} used by tests. */
  static LEDIO create(Constants.Mode mode, LEDConstants.Hardware hardware) {
    return switch (mode) {
      case REPLAY -> new LEDIO() {};
      case REAL, SIM -> switch (hardware) {
        case PWM -> new LEDIOPWM(
            LEDConstants.PWM_PORT,
            LEDConstants.LENGTH_PIXELS,
            LEDConstants.SWAP_RED_GREEN_FOR_WS2811);
        case CANDLE -> mode == Constants.Mode.REAL
            ? LEDIOCandle.forReal(LEDConstants.SWAP_RED_GREEN_FOR_WS2811)
            : LEDIOCandle.forSim(LEDConstants.SWAP_RED_GREEN_FOR_WS2811);
      };
    };
  }
}
