package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

/** Exclusive controller routing: port 1 (demo) takes precedence over port 0 (full controls). */
public final class DriverControls {
  public static final int FULL_PORT = 0;
  public static final int DEMO_PORT = 1;

  /** Actions are mapped once; full-only actions are unavailable on the demo controller. */
  public enum Action {
    INTAKE(false, hid -> hid.getLeftTriggerAxis() > 0.1),
    EJECT(false, XboxController::getLeftBumperButton),
    SHOOT(false, XboxController::getRightBumperButton),
    FACE_FORWARD(true, XboxController::getYButton),
    FACE_BACK(true, XboxController::getAButton),
    FACE_LEFT(true, XboxController::getXButton),
    FACE_RIGHT(true, XboxController::getBButton),
    DEMO_FACE_FORWARD(false, hid -> hid.getPOV() == 0),
    DEMO_FACE_BACK(false, hid -> hid.getPOV() == 180),
    DEMO_FACE_LEFT(false, hid -> hid.getPOV() == 270),
    DEMO_FACE_RIGHT(false, hid -> hid.getPOV() == 90),
    CREEP_FORWARD(true, hid -> hid.getPOV() == 0),
    CREEP_BACK(true, hid -> hid.getPOV() == 180),
    CREEP_LEFT(true, hid -> hid.getPOV() == 270),
    CREEP_RIGHT(true, hid -> hid.getPOV() == 90),
    RESET_HEADING(false, XboxController::getBButton),
    REVERSE_FEED(true, hid -> hid.getRightTriggerAxis() > 0.5),
    RESET_SIM(true, XboxController::getStartButton),
    CYCLE_LED_THEME(false, XboxController::getBackButton),
    CYCLE_LED_IDLE_EFFECT(false, XboxController::getStartButton);

    private final boolean fullOnly;
    private final Predicate<XboxController> pressed;

    Action(boolean fullOnly, Predicate<XboxController> pressed) {
      this.fullOnly = fullOnly;
      this.pressed = pressed;
    }
  }

  private final XboxController[] controllers = {
    new XboxController(FULL_PORT), new XboxController(DEMO_PORT)
  };

  /** Returns -1 when neither controller is connected; priority is connection-based, not motion. */
  public int activePort() {
    if (DriverStation.isJoystickConnected(DEMO_PORT)) {
      return DEMO_PORT;
    }
    return DriverStation.isJoystickConnected(FULL_PORT) ? FULL_PORT : -1;
  }

  /**
   * True when the controller currently in control is holding this action. Full-only actions only
   * count while the full controller has control. Cleanup code uses this to recompute outputs after
   * any edge, so a release from a source that just lost control never zeroes an action the new
   * controller is still holding.
   */
  public boolean isHeldByActiveController(Action action) {
    int port = activePort();
    if (port < 0 || (action.fullOnly && port == DEMO_PORT)) {
      return false;
    }
    return action.pressed.test(controllers[port]);
  }

  /**
   * Raw hold state of one specific port, ignoring control priority — the physical button edge
   * source for bindings (like the latched intake) that must distinguish "the operator pressed" from
   * "control priority changed while held". Reads the hardware state of that pad only; a
   * disconnected pad's reads are garbage, so callers must gate on connection themselves.
   */
  public boolean isHeldByPort(int port, Action action) {
    if (action.fullOnly && port == DEMO_PORT) {
      return false;
    }
    return action.pressed.test(controllers[port]);
  }

  public double leftX() {
    return axis(XboxController::getLeftX);
  }

  public double leftY() {
    return axis(XboxController::getLeftY);
  }

  public double rightX() {
    return axis(XboxController::getRightX);
  }

  private double axis(ToDoubleFunction<XboxController> read) {
    int port = activePort();
    return port < 0 ? 0.0 : read.applyAsDouble(controllers[port]);
  }

  public void whileHeld(Action action, Supplier<Command> command) {
    bind(action, command, true);
  }

  public void onPress(Action action, Supplier<Command> command) {
    bind(action, command, false);
  }

  /** Binds a rising edge only while the demo controller has control. */
  public void onDemoPress(Action action, Supplier<Command> command) {
    Trigger trigger =
        new Trigger(() -> activePort() == DEMO_PORT && action.pressed.test(controllers[DEMO_PORT]));
    trigger.onTrue(command.get().onlyWhile(() -> activePort() == DEMO_PORT));
  }

  /**
   * Runs the command once when the hold ends — a normal release, a disconnect, or a priority loss
   * all make the gated trigger go false. Unlike {@link #onPress}/{@link #whileHeld} the command is
   * NOT wrapped in {@code onlyWhile}: the press gate has already gone false by the time this fires,
   * and the cleanup must run exactly then. Note the scheduler silently drops commands that do not
   * run when disabled (WPILib {@code CommandScheduler.schedule}), so a cleanup that must land
   * during a disable — like clearing a latched motor request — needs {@code ignoringDisable(true)}
   * on the command; the falling edge is consumed either way and is not re-fired on re-enable.
   */
  public void onRelease(Action action, Supplier<Command> command) {
    for (int port = FULL_PORT; port <= DEMO_PORT; port++) {
      if (action.fullOnly && port == DEMO_PORT) {
        continue;
      }
      final int source = port;
      Trigger trigger =
          new Trigger(() -> activePort() == source && action.pressed.test(controllers[source]));
      trigger.onFalse(command.get());
    }
  }

  private void bind(Action action, Supplier<Command> factory, boolean held) {
    for (int port = FULL_PORT; port <= DEMO_PORT; port++) {
      if (action.fullOnly && port == DEMO_PORT) {
        continue;
      }
      final int source = port;
      Trigger trigger =
          new Trigger(() -> activePort() == source && action.pressed.test(controllers[source]));
      // Separate command instances per source. In particular, a one-shot homing command must
      // also stop if its controller disconnects or loses priority before reaching the hard stop.
      Command command = factory.get().onlyWhile(() -> activePort() == source);
      if (held) {
        trigger.whileTrue(command);
      } else {
        trigger.onTrue(command);
      }
    }
  }
}
