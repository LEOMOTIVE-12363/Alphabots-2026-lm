package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.HoodZeroCommand.HoodZeroCommand;
import frc.robot.commands.IntakeCommand.IntakeCommand;
import frc.robot.commands.IntakeDeployZeroCommand.IntakeDeployZeroCommand;
import frc.robot.commands.ManualCommand.Manual;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSimulation;
import frc.robot.simulation.MapleSimArena;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.FeedPath.FeedPath;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.DashboardControls;
import frc.robot.util.DriverControls;
import frc.robot.util.DriverControls.Action;
import frc.robot.util.OperatorStatus;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // D-pad up/down nudges Flywheel Speed Limit (%) by this much.
  private static final double FLYWHEEL_PERCENT_STEP = 1.0;

  // Subsystems
  private final Drive drive;

  // Touching each singleton here forces its class to initialize at boot, which runs the
  // constructor and flashes the motor config (gear ratio, PID, etc). Without an eager reference
  // these only initialize on first button press, so the config is never applied at deploy.
  private final Drum drum = Drum.mInstance;
  private final FeedPath feedPath = FeedPath.mInstance;
  private final Feeder feeder = Feeder.mInstance;
  private final Hood hood = Hood.mInstance;
  private final Indexer indexer = Indexer.mInstance;
  private final IntakeDeploy intakeDeploy = IntakeDeploy.mInstance;
  private final IntakeRoller intakeRoller = IntakeRoller.mInstance;
  private final LED led = LED.mInstance;

  // Port 1 is the preferred demo controller; port 0 retains the full original controls.
  private final DriverControls controls = new DriverControls();

  // Latched intake (2026-10-26): LT is press-on / press-off instead of hold-to-run. The latch
  // state is exactly "latchedIntake.isScheduled()" — the onlyWhile guard tears it down when its
  // source port disconnects or loses priority, and the scheduler cancels it on disable, so
  // neither resumes by itself; the operator presses LT again. intakeLatchSource records which
  // port owns the running latch for that guard.
  //
  // Edge semantics (2026-10-27 acceptance fix, ADR 0016): the toggle is driven by REAL button
  // edges polled every loop — not by a "has control AND pressed" trigger, whose condition
  // regaining truth masquerades as a press when control returns to a pad still holding LT.
  // A port whose latch was torn down while its LT was held must first OBSERVE a release before
  // its next press counts (intakeAwaitingRelease); while a pad is disconnected its button reads
  // are garbage, so its edge state is frozen and the reconnect loop only resynchronizes the
  // baseline — a hold that spanned the disconnect is not a new press either.
  private int intakeLatchSource = -1;
  private final Command latchedIntake =
      new IntakeCommand().onlyWhile(() -> controls.activePort() == intakeLatchSource);
  private final boolean[] intakeHeldLastLoop = new boolean[2];
  private final boolean[] intakeConnectedLastLoop = new boolean[2];
  private final boolean[] intakeAwaitingRelease = new boolean[2];
  private boolean intakeLatchWasScheduled;
  private int lastIntakeLatchSource = -1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        Drive.mInstance = drive;

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot: maple-sim arena + AdvantageKit drive IO (official hardware-abstraction path)
        var driveSimulation = MapleSimArena.getInstance().getDriveSimulation();
        FuelSimulation.configure(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        Drive.mInstance = drive;
        drive.setPose(MapleSimArena.getInstance().getRobotPose());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        Drive.mInstance = drive;
        break;
    }
    Drive.mInstance = drive;

    // Demo robot: no autonomous routines, no vision — teleop only.

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Bind shared and full-only actions, plus the demo pad's two LED selectors. */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controls.leftY() * 0.9,
            () -> -controls.leftX() * 0.9,
            () -> -controls.rightX()));
    hood.setDefaultCommand(
        Commands.run(
            () -> hood.setPositionRot(DashboardControls.getHoodTargetDeg() / 360.0), hood));

    controls.onPress(
        Action.RETRACT,
        () -> new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.INWARD));
    controls.onPress(Action.RESET_HEADING, this::resetHeadingCommand);
    // INTAKE (LT) is NOT bound here: its latch is driven by pollIntakeLatch()'s real-edge state
    // machine (see the latched-intake note above) so a control-priority change can never
    // masquerade as a fresh press.
    controls.whileHeld(Action.SHOOT, () -> new Shooting(false));
    controls.onPress(Action.EJECT, this::reverseFeedPressCommand);
    // The outtake press also drops the intake latch — see stopLatchedIntakeCommand for why this
    // is a separate poll-phase binding rather than part of applyReverseFeedDesired.
    controls.onPress(Action.EJECT, this::stopLatchedIntakeCommand);
    controls.onRelease(Action.EJECT, this::reverseFeedReleaseCommand);
    controls.whileHeld(Action.EJECT, () -> reverseFeedHeartbeatCommand(true));

    controls.onPress(
        Action.FLYWHEEL_UP,
        () ->
            Commands.runOnce(() -> DashboardControls.adjustFlywheelPercent(FLYWHEEL_PERCENT_STEP)));
    controls.onPress(
        Action.FLYWHEEL_DOWN,
        () ->
            Commands.runOnce(
                () -> DashboardControls.adjustFlywheelPercent(-FLYWHEEL_PERCENT_STEP)));
    controls.whileHeld(Action.MANUAL_SHOOT, Manual::new);
    controls.whileHeld(Action.CURRENT_FEED_SHOOT, () -> new Shooting(true));
    controls.onPress(Action.REVERSE_FEED, this::reverseFeedPressCommand);
    controls.onRelease(Action.REVERSE_FEED, this::reverseFeedReleaseCommand);
    controls.whileHeld(Action.REVERSE_FEED, () -> reverseFeedHeartbeatCommand(false));
    controls.onPress(
        Action.ZERO_OUTWARD,
        () ->
            new HoodZeroCommand()
                .alongWith(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.OUTWARD)));
    controls.onPress(
        Action.RESET_SIM,
        () ->
            Commands.runOnce(
                    () -> {
                      if (Constants.currentMode == Constants.Mode.SIM
                          && MapleSimArena.getInstance() != null) {
                        MapleSimArena.getInstance().resetField();
                      }
                    })
                .ignoringDisable(true));
    controls.onDemoPress(Action.CYCLE_LED_THEME, () -> Commands.runOnce(led::cycleTheme));
    controls.onDemoPress(
        Action.CYCLE_LED_IDLE_EFFECT, () -> Commands.runOnce(led::cycleIdleEffect));
  }

  private Command resetHeadingCommand() {
    return Commands.runOnce(
            () -> {
              boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              drive.setPose(
                  new Pose2d(
                      drive.getPose().getTranslation(),
                      isFlipped ? Rotation2d.kPi : Rotation2d.kZero));
            },
            drive)
        .ignoringDisable(true);
  }

  // Momentary reverse semantics, preserved from the original pre-DriverControls bindings: one-shot
  // motor writes with NO subsystem ownership, so a concurrently running Shooting loop overwrites
  // them — that documented conflict stays (2026-10-22 decision). Press and release both recompute
  // the outputs the controller in control is asking for (2026-10-23 acceptance fix): a same-loop
  // takeover used to run the old source's release after the new source's press and zero the motors
  // mid-hold, and a release/disconnect while disabled was silently dropped by the scheduler,
  // latching the reverse request past re-enable. The release binding runs while disabled so cleanup
  // always lands; the press binding does not, so nothing starts while disabled.
  //
  // The recompute must run in execute(), not initialize() (2026-10-24 acceptance fix): initialize
  // fires synchronously at schedule time, i.e. during the button-loop poll — before the execution
  // phase where an outgoing priority-lost Shooting/Manual hits isFinished() and its end() zeroes
  // the feeder/indexer/roller. Commands scheduled this loop iterate after long-running ones, so an
  // execute()-phase write lands after those end() cleanups and a takeover LB survives; a poll-phase
  // write gets clobbered (feeder read 0 V until re-press).
  private Command reverseFeedPressCommand() {
    return reverseFeedApplyCommand(false);
  }

  /**
   * Toggles the latched intake from a REAL LT press on {@code port}. Runs from the poll loop (not
   * from a scheduler trigger): schedule() and cancel() here take the scheduler's immediate path
   * because the poll runs outside the scheduler's execute phase, and isScheduled() reflects the
   * latch the moment this returns. A toggle while disabled is dropped by the scheduler before the
   * latch schedules, so nothing latches across a disable.
   */
  private void toggleLatchedIntake(int port) {
    if (latchedIntake.isScheduled()) {
      latchedIntake.cancel();
      intakeLatchSource = -1;
    } else {
      intakeLatchSource = port;
      lastIntakeLatchSource = port;
      latchedIntake.schedule();
      if (!latchedIntake.isScheduled()) {
        intakeLatchSource = -1; // scheduler dropped it (e.g. disabled): not latched
      }
    }
  }

  /**
   * Real-edge poll for the latched intake — call every robot loop after the scheduler runs. See the
   * latched-intake field note for the semantics: only a physical press observed while the pad is
   * connected AND in control toggles the latch; a latch torn down while LT was held (priority loss,
   * disconnect, disable, LB, interruption) requires an observed release first; a disconnect freezes
   * the edge state and a reconnect only resynchronizes it.
   */
  public void pollIntakeLatch() {
    for (int port = DriverControls.FULL_PORT; port <= DriverControls.DEMO_PORT; port++) {
      boolean connected = DriverStation.isJoystickConnected(port);
      boolean connectedLastLoop = intakeConnectedLastLoop[port];
      intakeConnectedLastLoop[port] = connected;
      if (!connected) {
        continue; // button reads are garbage while unplugged: freeze this port's edge state
      }
      boolean held = controls.isHeldByPort(port, Action.INTAKE);
      if (!connectedLastLoop) {
        // Reconnect loop: resynchronize the baseline only. A hold that spanned the disconnect is
        // not a new press; a press made during the disconnect becomes a press only on its next
        // real edge, which is the conservative reading.
        intakeHeldLastLoop[port] = held;
        continue;
      }
      boolean rising = held && !intakeHeldLastLoop[port];
      intakeHeldLastLoop[port] = held;
      if (!held) {
        intakeAwaitingRelease[port] = false; // a release observed while connected clears the gate
      }
      if (rising
          && !intakeAwaitingRelease[port]
          && controls.activePort() == port
          && !DriverStation.isDisabled()) {
        toggleLatchedIntake(port);
      }
    }

    if (intakeLatchWasScheduled && !latchedIntake.isScheduled() && lastIntakeLatchSource >= 0) {
      // The latch was torn down this loop (onlyWhile, disable, LB, interruption — or this poll's
      // own toggle-off, which is harmless). If the source pad still holds LT, require a release.
      if (intakeHeldLastLoop[lastIntakeLatchSource]) {
        intakeAwaitingRelease[lastIntakeLatchSource] = true;
      }
    }
    intakeLatchWasScheduled = latchedIntake.isScheduled();
  }

  /**
   * An outtake press takes over the feed path: cancel the latched intake in the button poll so its
   * end() zero-writes land BEFORE the execute-phase -3 V reverse recompute of
   * applyReverseFeedDesired. Cancelling from inside that recompute instead would defer the latch's
   * end() to loop end (WPILib 2026 queues in-loop cancels into m_toCancelCommands, processed after
   * the execute phase) and the zeros would overwrite the fresh reverse outputs.
   */
  private Command stopLatchedIntakeCommand() {
    return Commands.runOnce(
        () -> {
          if (latchedIntake.isScheduled()) {
            latchedIntake.cancel();
            intakeLatchSource = -1;
          }
        });
  }

  private Command reverseFeedReleaseCommand() {
    return reverseFeedApplyCommand(true);
  }

  /** One-shot that applies the reverse recompute in execute() — see the note above. */
  private Command reverseFeedApplyCommand(boolean runsWhenDisabled) {
    return new FunctionalCommand(
            () -> {}, this::applyReverseFeedDesired, interrupted -> {}, () -> true)
        .ignoringDisable(runsWhenDisabled);
  }

  /**
   * Write the reverse outputs requested by the controller currently in control: LB = full reverse
   * (either port), RT = local reverse (full port only). Whatever is not requested is zeroed, so
   * releasing one reverse action never kills the other, and the same-loop order of a press and a
   * losing source's release no longer matters. While disabled everything is zeroed — cleanup lands,
   * starts do not. The heartbeat bindings only drive the display layers.
   */
  private void applyReverseFeedDesired() {
    boolean full = controls.isHeldByActiveController(Action.EJECT);
    boolean local = controls.isHeldByActiveController(Action.REVERSE_FEED);
    if (DriverStation.isDisabled()) {
      full = false;
      local = false;
    }
    Indexer.mInstance.setV(full || local ? -3 : 0);
    Feeder.mInstance.setV(full || local ? -3 : 0);
    IntakeRoller.mInstance.setV(full ? -3 : 0);
  }

  private Command reverseFeedHeartbeatCommand(boolean includeIntake) {
    return Commands.run(
        () -> {
          BallFlowTracker.report(
              includeIntake
                  ? BallFlowTracker.FlowKind.FULL_REVERSE
                  : BallFlowTracker.FlowKind.LOCAL_REVERSE);
          OperatorStatus.report(
              includeIntake
                  ? OperatorStatus.Action.REVERSING_FULL
                  : OperatorStatus.Action.REVERSING_LOCAL);
        });
  }

  /**
   * Dashboard outputs refreshed every loop. The operator-status panel is published separately by
   * {@link Robot}'s 10 Hz block so it stays live no matter which controller-binding scheme this
   * class carries.
   */
  public void updateDashboardOutputs() {
    hood.publishDashboard();
    Logger.recordOutput("Controls/ActiveController", controls.activePort());
    if (!Logger.hasReplaySource()) {
      SmartDashboard.putBoolean(
          "Controller 0 Connected", DriverStation.isJoystickConnected(DriverControls.FULL_PORT));
      SmartDashboard.putNumber("Active Controller", controls.activePort());
    }
  }
}
