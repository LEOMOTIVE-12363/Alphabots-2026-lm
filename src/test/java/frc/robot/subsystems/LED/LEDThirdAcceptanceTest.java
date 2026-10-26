package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.junit.jupiter.api.Test;

class LEDThirdAcceptanceTest {
  @Test
  void periodicOutputMustNotWaitForConfigurationTimeout() {
    double[] t = {0};
    var bus =
        new LEDRestartAcceptanceTest.Bus() {
          @Override
          public StatusCode configureStrip(StripTypeValue s) {
            if (t[0] > 0) {
              try {
                Thread.sleep(100);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            }
            return super.configureStrip(s);
          }
        };
    bus.ready = true;
    var io = new LEDIOCandle(bus, 200, false, .05, 24, () -> t[0]);
    var frame = new AddressableLEDBuffer(200);
    io.writeBuffer(frame);
    t[0] = LEDConstants.CANDLE_REFRESH_INTERVAL_SEC + .01;
    long start = System.nanoTime();
    io.writeBuffer(frame);
    double elapsed = (System.nanoTime() - start) / 1e6;
    System.out.println("periodic write elapsed ms = " + elapsed);
    assertTrue(
        elapsed < 20,
        "CAN configuration latency must not stall the 20 ms robot loop: " + elapsed + " ms");
  }

  @Test
  void legacyChaseMustShareNormalizedPhaseWithOnboardThumbnail() {
    var strip = new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);
    var onboard = new AddressableLEDBuffer(8);
    // At one external-strip lap, both heads should return to normalized position zero.
    double now =
        (strip.getLength() + LEDConstants.CHASE_TAIL_PIXELS)
            / LEDConstants.CHASE_SPEED_PIXELS_PER_SEC;
    LED.renderLegacyInto(strip, LEDPattern.CHASE, now, 100);
    LED.renderLegacyInto(onboard, LEDPattern.CHASE, now, 100);
    assertEquals(
        strip.getLED(0), onboard.getLED(0), "Same-time Chase lap start should match at u=0");
  }
}
