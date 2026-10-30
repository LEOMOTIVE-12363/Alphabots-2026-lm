package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.io.MotorIO;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualCommand.Manual;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Integration acceptance for the dual-controller reverse wiring: the real RobotContainer bindings,
 * the real CommandScheduler, DriverStationSim, and the sim mechanism IO. Asserts on latched motor
 * REQUESTS (voltage setpoints) — this says nothing about physical motor motion while the robot is
 * disabled, which needs the real robot.
 *
 * <p>The container's bindings register on the default button loop once for the whole JVM. Other
 * test classes never assert on these motors' voltage requests, and every test here re-zeros them,
 * so the shared scheduler stays harmless cross-class chatter.
 */
class DriverControlsIntegrationAcceptanceTest {
  private static final int BUTTON_LB = 5;
  private static final int AXIS_RIGHT_TRIGGER = 3;
  private static final double EPS = 1e-6;

  private DriverControls controls;

  @BeforeAll
  static void setUpContainer() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    new RobotContainer();
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    loop();
    zeroReversePath();
    controls = new DriverControls();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    zeroReversePath();
  }

  @AfterAll
  static void tearDownClass() {
    // This class constructs a RobotContainer, which sets Drive.mInstance; ShootingTest asserts the
    // singleton is null, so hand the JVM back clean (same hygiene as TeleopIntegrationTest).
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    if (Drive.mInstance != null) {
      scheduler.unregisterSubsystem(Drive.mInstance);
      Drive.mInstance = null;
    }
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  private static void zeroReversePath() {
    Feeder.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    IntakeRoller.mInstance.setV(0);
  }

  private static void loop() {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run();
  }

  private static void connected(int port, boolean yes) {
    DriverStationSim.setJoystickIsXbox(port, yes);
    DriverStationSim.setJoystickAxisCount(port, yes ? 6 : 0);
    DriverStationSim.setJoystickButtonCount(port, yes ? 10 : 0);
    DriverStationSim.setJoystickPOVCount(port, yes ? 1 : 0);
    DriverStationSim.setJoystickPOV(port, 0, -1);
  }

  private static double feederVolts() {
    Feeder.mInstance.periodic();
    return Feeder.mInstance.getVoltageSetpoint();
  }

  private static double indexerVolts() {
    Indexer.mInstance.periodic();
    return Indexer.mInstance.getVoltageSetpoint();
  }

  private static double rollerVolts() {
    IntakeRoller.mInstance.periodic();
    return IntakeRoller.mInstance.getVoltageSetpoint();
  }

  @Test
  void acceptanceNormalReleaseAndDisconnectStopReverse() {
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, feederVolts(), EPS);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, false);
    loop();
    assertEquals(0, feederVolts(), EPS, "a normal release stops the eject");
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    connected(DriverControls.DEMO_PORT, false);
    loop();
    assertEquals(0, feederVolts(), EPS, "a mid-hold disconnect stops the eject");
  }

  @Test
  void acceptanceTakeoverKeepsNewOwnersReverse() {
    connected(DriverControls.FULL_PORT, true);
    loop();
    DriverStationSim.setJoystickAxis(DriverControls.FULL_PORT, AXIS_RIGHT_TRIGGER, 1.0);
    loop();
    assertEquals(-3, feederVolts(), EPS);
    connected(DriverControls.DEMO_PORT, true);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    loop();
    assertEquals(DriverControls.DEMO_PORT, controls.activePort());
    assertEquals(
        -3,
        feederVolts(),
        EPS,
        "new owner holds LB but the old owner's cleanup overwrote its reverse");
  }

  @Test
  void acceptanceFallbackKeepsNewOwnersReverse() {
    connected(DriverControls.FULL_PORT, true);
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickAxis(DriverControls.FULL_PORT, AXIS_RIGHT_TRIGGER, 1.0);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, feederVolts(), EPS);
    connected(DriverControls.DEMO_PORT, false);
    loop();
    loop();
    assertEquals(DriverControls.FULL_PORT, controls.activePort());
    assertEquals(
        -3,
        feederVolts(),
        EPS,
        "fallback controller holds RT but the losing source's cleanup overwrote it");
  }

  @Test
  void acceptanceDisabledDisconnectClearsReverseBeforeReenable() {
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, feederVolts(), EPS);
    DriverStationSim.setEnabled(false);
    loop();
    connected(DriverControls.DEMO_PORT, false);
    loop();
    DriverStationSim.setEnabled(true);
    loop();
    assertEquals(-1, controls.activePort());
    assertEquals(
        0,
        feederVolts(),
        EPS,
        "no controller is connected but the reverse request was never cleared");
  }

  @Test
  void acceptanceReleaseWhileDisabledClearsReverse() {
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, feederVolts(), EPS);
    DriverStationSim.setEnabled(false);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, false);
    loop();
    assertEquals(0, feederVolts(), EPS, "a release while disabled must still clear the request");
    DriverStationSim.setEnabled(true);
    loop();
    assertEquals(0, feederVolts(), EPS, "re-enable does not resurrect the eject");
  }

  @Test
  void acceptancePressWhileDisabledDoesNotStart() {
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setEnabled(false);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(0, feederVolts(), EPS, "a press while disabled must not start the eject");
    DriverStationSim.setEnabled(true);
    loop();
    assertEquals(0, feederVolts(), EPS, "the consumed edge does not fire on re-enable");
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, false);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, feederVolts(), EPS, "a fresh press after re-enable starts the eject");
  }

  @Test
  void acceptanceTakeoverFromManualKeepsNewOwnersReverse() {
    // Directly scheduled Manual retains the old port-0 ownership gate. Port 1 connects already
    // holding LB: outgoing end() zeroes the path in the same loop, and the new reverse must win.
    connected(DriverControls.FULL_PORT, true);
    loop();
    new Manual().onlyWhile(() -> controls.activePort() == DriverControls.FULL_PORT).schedule();
    loop();
    assertTakeoverKeepsNewOwnersReverse();
  }

  @Test
  void acceptanceTakeoverFromCurrentFeedKeepsNewOwnersReverse() {
    // Same takeover, but the outgoing command is directly scheduled Shooting(true) (current-feed).
    connected(DriverControls.FULL_PORT, true);
    loop();
    new Shooting(true)
        .onlyWhile(() -> controls.activePort() == DriverControls.FULL_PORT)
        .schedule();
    loop();
    for (int i = 0;
        i < 40 && Feeder.mInstance.getControlMode() != MotorIO.ControlMode.CURRENT;
        i++) {
      SimHooks.stepTiming(0.02);
      loop();
    }
    assertEquals(
        MotorIO.ControlMode.CURRENT,
        Feeder.mInstance.getControlMode(),
        "outgoing Shooting(true) must establish current-feed before takeover");
    assertEquals(50.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS);
    assertTakeoverKeepsNewOwnersReverse();
  }

  private void assertTakeoverKeepsNewOwnersReverse() {
    connected(DriverControls.DEMO_PORT, true);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    loop();
    assertEquals(DriverControls.DEMO_PORT, controls.activePort(), "port 1 takes over");
    double[] atHandover = reversePathVolts();
    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(0.02);
      loop();
    }
    double[] afterOneSecond = reversePathVolts();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, false);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    double[] afterRepress = reversePathVolts();
    assertAll(
        () ->
            assertEquals(
                -3, atHandover[0], EPS, "feeder: outgoing command's end() must not win the loop"),
        () -> assertEquals(-3, atHandover[1], EPS, "indexer: same handoff"),
        () -> assertEquals(-3, atHandover[2], EPS, "intake roller: same handoff"),
        () -> assertEquals(-3, afterOneSecond[0], EPS, "feeder: reverse retained while LB held"),
        () -> assertEquals(-3, afterOneSecond[1], EPS, "indexer: retained while held"),
        () -> assertEquals(-3, afterOneSecond[2], EPS, "intake roller: retained while held"),
        () -> assertEquals(-3, afterRepress[0], EPS, "feeder: re-press also works"),
        () -> assertEquals(-3, afterRepress[1], EPS, "indexer: re-press"),
        () -> assertEquals(-3, afterRepress[2], EPS, "intake roller: re-press"));
  }

  /** Feeder, indexer, intake roller latched voltage requests, in that order. */
  private static double[] reversePathVolts() {
    return new double[] {feederVolts(), indexerVolts(), rollerVolts()};
  }

  @Test
  void acceptanceFullRtRunsFullReverseAndFullLbDoesNotReverse() {
    connected(DriverControls.FULL_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.FULL_PORT, BUTTON_LB, true);
    loop();
    assertEquals(0, indexerVolts(), EPS, "LB is inward homing, not reverse");
    assertEquals(0, feederVolts(), EPS);
    assertEquals(0, rollerVolts(), EPS);
    DriverStationSim.setJoystickAxis(DriverControls.FULL_PORT, AXIS_RIGHT_TRIGGER, 1.0);
    loop();
    assertEquals(-3, indexerVolts(), EPS, "RT runs the complete reverse");
    assertEquals(-3, feederVolts(), EPS);
    assertEquals(-3, rollerVolts(), EPS, "RT also reverses the intake roller");
    DriverStationSim.setJoystickButton(DriverControls.FULL_PORT, BUTTON_LB, false);
    loop();
    assertEquals(-3, indexerVolts(), EPS, "releasing LB must not clear the RT reverse");
    assertEquals(-3, feederVolts(), EPS);
    assertEquals(-3, rollerVolts(), EPS);
    DriverStationSim.setJoystickAxis(DriverControls.FULL_PORT, AXIS_RIGHT_TRIGGER, 0.0);
    loop();
    assertEquals(0, indexerVolts(), EPS);
    assertEquals(0, feederVolts(), EPS);
    assertEquals(0, rollerVolts(), EPS);
  }
}
