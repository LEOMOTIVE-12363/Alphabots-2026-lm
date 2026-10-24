package frc.robot.util;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Hood.Hood;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * ShotReadiness against the sim mechanism IO: with instant setpoints the drum/hood snap to the
 * commanded setpoint on the next periodic(), so the readiness rule and its debounce can be checked
 * deterministically. The subsystem singletons are JVM-shared, so every test first commands a known
 * drum velocity and hood position and settles a few loops before evaluating.
 */
class ShotReadinessTest {
  private static final double HOOD_SETTLE_ROT = 0.05;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  void resetDashboard() {
    DashboardControlsTestUtil.resetDefaults();
  }

  /** One robot loop: advance time, then update mechanism inputs. */
  private static void step() {
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
  }

  /** Commands the drum to full shot speed and the hood to a known angle, then settles the sim. */
  private static void commandAndSettle() {
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.MaxShotVelocity);
    Hood.mInstance.setPositionRot(HOOD_SETTLE_ROT);
    for (int i = 0; i < 3; i++) {
      step();
    }
  }

  @Test
  void rawReadyImmediatelyAndStableAfterDebounce() {
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    commandAndSettle();

    // Hood target = the settled position, so only the debounce gates stableReady.
    ShotReadiness readiness = new ShotReadiness();
    ShotReadiness.Evaluation eval =
        readiness.update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
    assertFalse(eval.flywheelZero());
    assertTrue(eval.drumReady());
    assertTrue(eval.hoodReady());
    assertTrue(eval.rawReady(), "ready on the first evaluation after settling");
    assertEquals(ShotReadiness.REASON_NONE, eval.blockReason());
    assertFalse(eval.stableReady(), "debounce has not seen 0.15 s of readiness yet");

    // 6 more loops = 0.12 s of continuous readiness: still inside the debounce window.
    for (int i = 0; i < 6; i++) {
      step();
      eval = readiness.update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
      assertTrue(eval.rawReady());
      assertFalse(eval.stableReady(), "must not pass before the full debounce");
    }

    // 4 more loops = 0.20 s total: past the 0.15 s debounce.
    for (int i = 0; i < 4; i++) {
      step();
      eval = readiness.update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
    }
    assertTrue(eval.rawReady());
    assertTrue(eval.stableReady(), "held ready for the full debounce");
  }

  @Test
  void zeroFlywheelBlocksWithFlywheelReason() {
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    commandAndSettle();

    ShotReadiness.Evaluation eval =
        new ShotReadiness().update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
    assertTrue(eval.flywheelZero());
    assertFalse(eval.drumReady(), "drum never reports at-speed while the limit holds it at zero");
    assertFalse(eval.rawReady());
    assertFalse(eval.stableReady());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, eval.blockReason());
  }

  @Test
  void drumOffSpeedRemainsATruthfulDiagnostic() {
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    commandAndSettle();

    ShotReadiness.Evaluation eval =
        new ShotReadiness()
            .update(DrumConstants.MaxShotVelocity + 7.0, Hood.mInstance.getPositionRot());
    assertFalse(eval.flywheelZero());
    assertFalse(eval.drumReady(), "the measured speed must not be faked ready");
    assertTrue(eval.hoodReady());
    assertFalse(eval.rawReady(), "raw readiness remains a measurement diagnostic");
    assertFalse(eval.stableReady());
    assertEquals(ShotReadiness.REASON_DRUM_NOT_AT_SPEED, eval.blockReason());
  }

  @Test
  void hoodOffTargetRemainsATruthfulDiagnostic() {
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    commandAndSettle();

    // Far outside HOOD_ANGLE_TOLERANCE_ROT (0.02) from the settled position.
    double farTargetRot = Hood.mInstance.getPositionRot() + 0.5;
    ShotReadiness.Evaluation eval =
        new ShotReadiness().update(DrumConstants.MaxShotVelocity, farTargetRot);
    assertFalse(eval.flywheelZero());
    assertTrue(eval.drumReady(), "flywheel ready, so the hood diagnostic surfaces");
    assertFalse(eval.hoodReady(), "the measured angle must not be faked ready");
    assertFalse(eval.rawReady(), "raw readiness remains a measurement diagnostic");
    assertFalse(eval.stableReady());
    assertEquals(ShotReadiness.REASON_HOOD_NOT_AT_ANGLE, eval.blockReason());
  }

  @Test
  void resetRestartsTheDebounce() {
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    commandAndSettle();

    ShotReadiness readiness = new ShotReadiness();
    ShotReadiness.Evaluation eval =
        readiness.update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
    for (int i = 0; i < 10; i++) {
      step();
      eval = readiness.update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
    }
    assertTrue(eval.stableReady(), "held ready past the debounce");

    readiness.reset();
    eval = readiness.update(DrumConstants.MaxShotVelocity, Hood.mInstance.getPositionRot());
    assertTrue(eval.rawReady(), "raw readiness is unaffected by reset");
    assertFalse(eval.stableReady(), "reset restarts the debounce from zero");
  }
}
