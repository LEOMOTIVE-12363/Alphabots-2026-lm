package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.DriverControls.Action;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Queue;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * DriverControls port priority and command-lifetime routing, driven through DriverStationSim on the
 * default CommandScheduler (same harness pattern as OperatorStatusTest).
 *
 * <p>Triggers poll on the default scheduler's button loop and live for the whole JVM once
 * registered, so every test builds a fresh DriverControls with fresh probes and only asserts on its
 * own probes — a stale trigger from an earlier test can re-run that test's dead probe, which is
 * harmless.
 */
class DriverControlsTest {
  // Xbox layout: buttons A=1 B=2 X=3 Y=4 LB=5 RB=6 Start=8; axes 0=leftX 1=leftY 2=LT 3=RT
  // 4=rightX 5=rightY.
  private static final int BUTTON_A = 1;
  private static final int BUTTON_X = 3;
  private static final int BUTTON_Y = 4;
  private static final int BUTTON_LB = 5;
  private static final int BUTTON_RB = 6;
  private static final int BUTTON_START = 8;
  private static final int AXIS_LEFT_Y = 1;
  private static final int AXIS_RIGHT_TRIGGER = 3;
  private static final int POV_INDEX = 0;

  /** Probe command: counts initialize() calls and records the end(interrupted) argument. */
  private static final class Probe extends Command {
    private final boolean finishImmediately;
    int initializeCount = 0;
    Boolean endedInterrupted = null; // null until end() runs

    Probe(boolean finishImmediately) {
      this.finishImmediately = finishImmediately;
    }

    @Override
    public void initialize() {
      initializeCount++;
    }

    @Override
    public boolean isFinished() {
      return finishImmediately;
    }

    @Override
    public void end(boolean interrupted) {
      endedInterrupted = interrupted;
    }

    boolean started() {
      return initializeCount > 0;
    }

    boolean running() {
      return started() && endedInterrupted == null;
    }
  }

  /**
   * DriverControls calls the factory once per allowed port in port order (0 then 1), so handing out
   * probes from this queue identifies which port's command instance fired.
   */
  private static final class ProbeFactory implements Supplier<Command> {
    private final Queue<Probe> probes;

    ProbeFactory(Probe... probesInPortOrder) {
      probes = new ArrayDeque<>(Arrays.asList(probesInPortOrder));
    }

    @Override
    public Command get() {
      return probes.remove();
    }
  }

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    resetJoystick(DriverControls.FULL_PORT);
    resetJoystick(DriverControls.DEMO_PORT);
    // Enabled + attached so commands that do not ignore disable are allowed to schedule.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    resetJoystick(DriverControls.FULL_PORT);
    resetJoystick(DriverControls.DEMO_PORT);
    // Disabled, disconnected baseline so no driver-station state leaks into other test classes.
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
  }

  /**
   * One robot loop for the binding layer: publish sim state, then poll triggers and run commands.
   */
  private static void runLoop() {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run();
  }

  private static void connect(int port) {
    DriverStationSim.setJoystickIsXbox(port, true);
    DriverStationSim.setJoystickAxisCount(port, 6);
    DriverStationSim.setJoystickButtonCount(port, 10);
    DriverStationSim.setJoystickPOVCount(port, 1);
    runLoop();
  }

  private static void disconnect(int port) {
    resetJoystick(port);
    runLoop();
  }

  /** Neutral inputs and zeroed connection counts: the port reads as fully disconnected. */
  private static void resetJoystick(int port) {
    neutralizeJoystick(port);
    DriverStationSim.setJoystickIsXbox(port, false);
    DriverStationSim.setJoystickAxisCount(port, 0);
    DriverStationSim.setJoystickButtonCount(port, 0);
    DriverStationSim.setJoystickPOVCount(port, 0);
  }

  /** Zero every input without touching the connection counts. */
  private static void neutralizeJoystick(int port) {
    for (int axis = 0; axis < 6; axis++) {
      DriverStationSim.setJoystickAxis(port, axis, 0.0);
    }
    for (int button = 1; button <= 10; button++) {
      DriverStationSim.setJoystickButton(port, button, false);
    }
    DriverStationSim.setJoystickPOV(port, POV_INDEX, -1);
  }

  private static void pressButton(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, true);
    runLoop();
  }

  private static void releaseButton(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, false);
    runLoop();
  }

  private static void setPov(int port, int degrees) {
    DriverStationSim.setJoystickPOV(port, POV_INDEX, degrees);
    runLoop();
  }

  @Test
  void connectionBasedPriorityAndAxisRouting() {
    DriverControls controls = new DriverControls();

    // Neither connected: no active port and every drive axis reads zero.
    assertEquals(-1, controls.activePort());
    assertEquals(0.0, controls.leftX());
    assertEquals(0.0, controls.leftY());
    assertEquals(0.0, controls.rightX());

    connect(DriverControls.FULL_PORT);
    assertTrue(
        DriverStation.isJoystickConnected(DriverControls.FULL_PORT),
        "sanity: sim connection plumbing makes the port report connected");
    assertEquals(DriverControls.FULL_PORT, controls.activePort(), "only port 0 connected");

    connect(DriverControls.DEMO_PORT);
    assertEquals(
        DriverControls.DEMO_PORT, controls.activePort(), "port 1 wins when both are connected");

    // With both connected the drive axes come from port 1.
    DriverStationSim.setJoystickAxis(DriverControls.FULL_PORT, AXIS_LEFT_Y, 0.25);
    DriverStationSim.setJoystickAxis(DriverControls.DEMO_PORT, AXIS_LEFT_Y, -0.75);
    DriverStationSim.notifyNewData();
    assertEquals(-0.75, controls.leftY(), 1e-9, "both connected: port 1's axis wins");

    // Losing port 1 falls back to port 0.
    disconnect(DriverControls.DEMO_PORT);
    assertEquals(DriverControls.FULL_PORT, controls.activePort());
    assertEquals(0.25, controls.leftY(), 1e-9, "port 1 gone: falls back to port 0");

    // Only port 1 connected.
    disconnect(DriverControls.FULL_PORT);
    connect(DriverControls.DEMO_PORT);
    assertEquals(DriverControls.DEMO_PORT, controls.activePort(), "only port 1 connected");

    // Both gone: back to no active port and zeroed inputs.
    disconnect(DriverControls.DEMO_PORT);
    assertEquals(-1, controls.activePort());
    assertEquals(0.0, controls.leftX());
    assertEquals(0.0, controls.leftY());
    assertEquals(0.0, controls.rightX());
  }

  @Test
  void sharedActionFiresFromEitherPortWhenItIsTheOnlyOneConnected() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(true);
    Probe port1 = new Probe(true);
    controls.onPress(Action.SHOOT, new ProbeFactory(port0, port1));

    connect(DriverControls.FULL_PORT);
    pressButton(DriverControls.FULL_PORT, BUTTON_RB);
    assertEquals(1, port0.initializeCount, "port 0's instance fires when it is alone");
    assertFalse(port1.started());

    releaseButton(DriverControls.FULL_PORT, BUTTON_RB);
    disconnect(DriverControls.FULL_PORT);
    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.DEMO_PORT, BUTTON_RB);
    assertEquals(1, port1.initializeCount, "port 1's instance fires when it is alone");
    assertEquals(1, port0.initializeCount, "port 0's instance did not fire again");
  }

  @Test
  void fullOnlyActionsNeverFireOnDemoPort() {
    DriverControls controls = new DriverControls();
    Probe a = new Probe(true);
    Probe retract0 = new Probe(true);
    Probe retract1 = new Probe(true);
    Probe y = new Probe(true);
    Probe start = new Probe(true);
    Probe rightTrigger = new Probe(true);
    controls.onPress(Action.CURRENT_FEED_SHOOT, new ProbeFactory(a));
    controls.onPress(Action.RETRACT, new ProbeFactory(retract0, retract1));
    controls.onPress(Action.ZERO_OUTWARD, new ProbeFactory(y));
    controls.onPress(Action.RESET_SIM, new ProbeFactory(start));
    controls.onPress(Action.REVERSE_FEED, new ProbeFactory(rightTrigger));

    // Full-only inputs on port 1 never fire; cardinal POVs must not fire RETRACT either.
    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.DEMO_PORT, BUTTON_A);
    pressButton(DriverControls.DEMO_PORT, BUTTON_X);
    pressButton(DriverControls.DEMO_PORT, BUTTON_Y);
    pressButton(DriverControls.DEMO_PORT, BUTTON_START);
    setPov(DriverControls.DEMO_PORT, 0);
    setPov(DriverControls.DEMO_PORT, 180);
    setPov(DriverControls.DEMO_PORT, 270);
    setPov(DriverControls.DEMO_PORT, -1);
    DriverStationSim.setJoystickAxis(DriverControls.DEMO_PORT, AXIS_RIGHT_TRIGGER, 1.0);
    runLoop();
    assertFalse(a.started(), "A must not fire on port 1");
    assertFalse(retract0.started(), "port 1 X must not fire the port 0 RETRACT instance");
    assertFalse(retract1.started(), "RETRACT is full-only X, not a demo POV action");
    assertFalse(y.started(), "Y must not fire on port 1");
    assertFalse(start.started(), "the RESET_SIM Start binding must not fire on port 1");
    assertFalse(rightTrigger.started(), "RT must not fire on port 1");

    // The same full-only inputs on port 0 fire; POV left is not another RETRACT binding.
    disconnect(DriverControls.DEMO_PORT);
    connect(DriverControls.FULL_PORT);
    pressButton(DriverControls.FULL_PORT, BUTTON_A);
    pressButton(DriverControls.FULL_PORT, BUTTON_X);
    pressButton(DriverControls.FULL_PORT, BUTTON_Y);
    pressButton(DriverControls.FULL_PORT, BUTTON_START);
    setPov(DriverControls.FULL_PORT, 0);
    setPov(DriverControls.FULL_PORT, 180);
    setPov(DriverControls.FULL_PORT, 270);
    setPov(DriverControls.FULL_PORT, -1);
    DriverStationSim.setJoystickAxis(DriverControls.FULL_PORT, AXIS_RIGHT_TRIGGER, 1.0);
    runLoop();
    assertEquals(1, a.initializeCount);
    assertEquals(1, retract0.initializeCount, "port 0 X fires RETRACT exactly once");
    assertFalse(retract1.started(), "the demo RETRACT instance stays unused");
    assertEquals(1, y.initializeCount);
    assertEquals(1, start.initializeCount);
    assertEquals(1, rightTrigger.initializeCount);
  }

  @Test
  void inputFromNonPriorityPortIsIgnored() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(true);
    Probe port1 = new Probe(true);
    controls.onPress(Action.SHOOT, new ProbeFactory(port0, port1));

    connect(DriverControls.FULL_PORT);
    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.FULL_PORT, BUTTON_RB);
    assertFalse(port0.started(), "port 0 has no control while port 1 is connected");
    assertFalse(port1.started(), "port 1's instance must not fire for port 0's button");
  }

  @Test
  void whileHeldStopsOnRelease() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(false);
    Probe port1 = new Probe(false);
    controls.whileHeld(Action.SHOOT, new ProbeFactory(port0, port1));

    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.DEMO_PORT, BUTTON_RB);
    assertTrue(port1.running(), "held command started");
    releaseButton(DriverControls.DEMO_PORT, BUTTON_RB);
    assertEquals(Boolean.TRUE, port1.endedInterrupted, "release interrupts the held command");
  }

  @Test
  void whileHeldStopsOnDisconnect() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(false);
    Probe port1 = new Probe(false);
    controls.whileHeld(Action.SHOOT, new ProbeFactory(port0, port1));

    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.DEMO_PORT, BUTTON_RB);
    assertTrue(port1.running(), "held command started");
    disconnect(DriverControls.DEMO_PORT);
    assertEquals(Boolean.TRUE, port1.endedInterrupted, "disconnect interrupts the held command");
  }

  @Test
  void prioritySwitchCancelsPort0AndHandsOverToPort1() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(false);
    Probe port1 = new Probe(false);
    controls.whileHeld(Action.SHOOT, new ProbeFactory(port0, port1));

    connect(DriverControls.FULL_PORT);
    pressButton(DriverControls.FULL_PORT, BUTTON_RB);
    assertTrue(port0.running(), "port 0's command runs while it has control");

    connect(DriverControls.DEMO_PORT);
    assertEquals(
        Boolean.TRUE, port0.endedInterrupted, "losing priority interrupts port 0's command");
    assertFalse(port1.started(), "port 1's command does not start by itself");

    pressButton(DriverControls.DEMO_PORT, BUTTON_RB);
    assertTrue(port1.running(), "port 1 now drives the action");
  }

  @Test
  void retractOneShotLongCommandCancelsOnDisconnect() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(false);
    Probe port1 = new Probe(false);
    controls.onPress(Action.RETRACT, new ProbeFactory(port0, port1));

    connect(DriverControls.FULL_PORT);
    pressButton(DriverControls.FULL_PORT, BUTTON_X);
    assertTrue(port0.running(), "long inward homing one-shot started via port 0 X");
    disconnect(DriverControls.FULL_PORT);
    assertEquals(Boolean.TRUE, port0.endedInterrupted, "disconnect interrupts the one-shot");
  }

  @Test
  void releaseBindingRunsCleanupOnReleaseAndOnDisconnect() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(true);
    Probe port1 = new Probe(true);
    controls.onRelease(Action.EJECT, new ProbeFactory(port0, port1));

    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.DEMO_PORT, BUTTON_LB);
    assertFalse(port1.started(), "release command must not fire while held");
    releaseButton(DriverControls.DEMO_PORT, BUTTON_LB);
    assertEquals(1, port1.initializeCount, "a normal release runs the cleanup");
    assertEquals(Boolean.FALSE, port1.endedInterrupted, "the cleanup runs to completion");

    pressButton(DriverControls.DEMO_PORT, BUTTON_LB);
    disconnect(DriverControls.DEMO_PORT);
    assertEquals(2, port1.initializeCount, "losing the controller mid-hold also runs the cleanup");
    assertEquals(Boolean.FALSE, port1.endedInterrupted);
  }

  @Test
  void disableCancelsAHeldCommand() {
    DriverControls controls = new DriverControls();
    Probe port0 = new Probe(false);
    Probe port1 = new Probe(false);
    controls.whileHeld(Action.SHOOT, new ProbeFactory(port0, port1));

    connect(DriverControls.DEMO_PORT);
    pressButton(DriverControls.DEMO_PORT, BUTTON_RB);
    assertTrue(port1.running(), "held command started while enabled");

    DriverStationSim.setEnabled(false);
    runLoop();
    assertEquals(Boolean.TRUE, port1.endedInterrupted, "disable interrupts the held command");
  }
}
