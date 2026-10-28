package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

/**
 * State-machine contract for the startup steer-alignment gate: drive output is held at zero only
 * when a TRANSLATION command (never rotation-only, never one the caller flagged as manual rotation)
 * starts from MEASURED REST, released once every steer angle is within tolerance of its setpoint or
 * after a bounded timeout, and never re-armed while the robot keeps moving — a one-loop zero
 * command mid-drive followed by a direction change must not add a wait. reset() (disable) drops all
 * state so the next enable rebuilds from actual rest.
 */
class StartupAlignGateTest {

  @Test
  void rotationOnlyCommandNeverHoldsOrArms() {
    var gate = new StartupAlignGate();
    assertFalse(gate.shouldHoldDrive(0.0, false, true, true, false));
    assertFalse(gate.shouldHoldDrive(0.02, false, true, true, false));
    assertFalse(gate.shouldHoldDrive(0.04, false, true, true, false));
    // A following translation start is still a fresh edge and arms normally.
    assertTrue(gate.shouldHoldDrive(0.06, true, true, true, false), "edge after rotation arms");
  }

  @Test
  void manualRotationFlagNeverHoldsOrArmsEvenWithTranslation() {
    var activelyHeld = new StartupAlignGate();
    assertTrue(activelyHeld.shouldHoldDrive(0.0, true, true, true, false));
    assertFalse(
        activelyHeld.shouldHoldDrive(0.02, true, false, true, false),
        "manual takeover drops an active hold immediately");

    var gate = new StartupAlignGate();
    assertFalse(gate.shouldHoldDrive(0.0, true, false, true, false));
    assertFalse(gate.shouldHoldDrive(0.02, true, false, true, false));
    // Manual flag drops mid-hold semantics entirely; a later non-manual translation is a fresh
    // edge that arms only from measured rest.
    assertFalse(
        gate.shouldHoldDrive(0.04, true, true, false, false),
        "edge while rolling (not at rest) must not arm");
  }

  @Test
  void edgeWhileRollingNeverArms() {
    // The mid-drive direction-change flaw: one zero-command loop while the robot is still moving,
    // then a nonzero translation edge. Measured speed at that edge is far above rest, so no hold.
    var gate = new StartupAlignGate();
    assertTrue(gate.shouldHoldDrive(0.0, true, true, true, false), "rest edge arms the hold");
    assertFalse(gate.shouldHoldDrive(0.02, true, true, true, true)); // released by alignment
    assertFalse(gate.shouldHoldDrive(0.04, false, true, false, true)); // one zero loop, rolling
    assertFalse(
        gate.shouldHoldDrive(0.06, true, true, false, false),
        "re-arming a rolling direction change is forbidden");
    assertFalse(gate.shouldHoldDrive(0.08, true, true, false, false));
  }

  @Test
  void holdsAfterRestStartupUntilAligned() {
    var gate = new StartupAlignGate();
    assertFalse(gate.shouldHoldDrive(0.0, false, true, true, true)); // rest, aligned, no command
    assertTrue(gate.shouldHoldDrive(0.02, true, true, true, false), "rest edge arms the hold");
    assertTrue(gate.shouldHoldDrive(0.04, true, true, true, false), "hold continues misaligned");
    assertFalse(gate.shouldHoldDrive(0.06, true, true, true, true), "alignment releases");
  }

  @Test
  void releasedHoldStaysReleasedForTheRestOfTheRun() {
    var gate = new StartupAlignGate();
    gate.shouldHoldDrive(0.0, false, true, true, true);
    assertTrue(gate.shouldHoldDrive(0.02, true, true, true, false));
    assertFalse(gate.shouldHoldDrive(0.04, true, true, true, true)); // released
    assertFalse(
        gate.shouldHoldDrive(0.06, true, true, true, false),
        "misaligned again mid-command (still moving): no re-arm");
    assertFalse(gate.shouldHoldDrive(0.08, true, true, true, false));
  }

  @Test
  void holdTimesOutWhenAlignmentNeverCompletes() {
    var gate = new StartupAlignGate();
    gate.shouldHoldDrive(0.0, false, true, true, true);
    assertTrue(gate.shouldHoldDrive(0.02, true, true, true, false));
    double justBeforeTimeout = 0.02 + StartupAlignGate.ALIGN_TIMEOUT_SEC - 0.01;
    double justAfterTimeout = 0.02 + StartupAlignGate.ALIGN_TIMEOUT_SEC + 0.01;
    assertTrue(
        gate.shouldHoldDrive(justBeforeTimeout, true, true, true, false),
        "still holding just before the bounded timeout");
    assertFalse(
        gate.shouldHoldDrive(justAfterTimeout, true, true, true, false),
        "timeout releases the hold even though the wheels never aligned");
    assertFalse(
        gate.shouldHoldDrive(justAfterTimeout + 0.02, true, true, true, false),
        "the timeout release is final for this run");
  }

  @Test
  void zeroCommandRearmsOnlyFromMeasuredRest() {
    var gate = new StartupAlignGate();
    gate.shouldHoldDrive(0.0, false, true, true, true);
    assertTrue(gate.shouldHoldDrive(0.02, true, true, true, false));
    assertFalse(gate.shouldHoldDrive(0.04, true, true, true, true)); // released by alignment
    assertFalse(gate.shouldHoldDrive(0.06, false, true, false, true)); // parked again (at rest)
    assertTrue(
        gate.shouldHoldDrive(0.08, true, true, true, false),
        "the next rest-to-translation start is a fresh startup and holds again");
  }

  @Test
  void resetDropsStateSoContinuousCommandBehavesAsFreshStartup() {
    // The disable-mid-drive flaw: the gate remembered commandedLastLoop=true across the disable,
    // so a re-enabled held stick never re-armed. reset() must clear that.
    var gate = new StartupAlignGate();
    gate.shouldHoldDrive(0.0, true, true, true, false); // commanded, rolling/holding state
    assertTrue(gate.shouldHoldDrive(0.02, true, true, true, false));
    gate.reset(); // disable
    assertTrue(
        gate.shouldHoldDrive(0.04, true, true, true, false),
        "after reset, a continuing translation command from rest re-arms like a fresh start");
  }

  @Test
  void optimizedSteerErrorHandlesReversalBoundary() {
    // Exactly opposite: driven backwards, counts as aligned (error 0).
    assertEquals(
        0.0, StartupAlignGate.optimizedSteerErrorRad(Rotation2d.kPi, Rotation2d.kZero), 1e-9);
    assertEquals(
        0.0, StartupAlignGate.optimizedSteerErrorRad(Rotation2d.kZero, Rotation2d.kPi), 1e-9);
    // Just off opposite: the tiny angular distance to the mirrored angle.
    assertEquals(
        0.001,
        StartupAlignGate.optimizedSteerErrorRad(
            Rotation2d.fromRadians(Math.PI - 0.001), Rotation2d.kZero),
        1e-9);
    // Around the 90-degree optimize boundary: symmetric on both sides.
    assertEquals(
        Math.PI / 2 - 0.01,
        StartupAlignGate.optimizedSteerErrorRad(
            Rotation2d.fromRadians(Math.PI / 2 - 0.01), Rotation2d.kZero),
        1e-9);
    assertEquals(
        Math.PI / 2 - 0.01,
        StartupAlignGate.optimizedSteerErrorRad(
            Rotation2d.fromRadians(Math.PI / 2 + 0.01), Rotation2d.kZero),
        1e-9);
    // Wrap-around: -pi vs +pi is zero, not 2*pi.
    assertEquals(
        0.0,
        StartupAlignGate.optimizedSteerErrorRad(
            Rotation2d.fromRadians(Math.PI), Rotation2d.fromRadians(-Math.PI)),
        1e-9);
  }
}
