package frc.robot.commands.ManualCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.OperatorStatus;
import frc.robot.util.ShotReadiness;

/**
 * X-button fixed-setpoint shot path. Every execute drives the drum and hood to the current Manual
 * targets; feeding runs only while the flywheel is actually at the Manual speed target (stable
 * after the 0.15 s window) and the flywheel limit is positive. The hood never gates feeding. A zero
 * flywheel limit or a below-speed measurement stops only the feed path for that loop, and feeding
 * resumes by itself once the shot is stable-ready again.
 */
public class Manual extends Command {
  // Shared readiness rule with this command's own debounce, so a fresh press re-earns the full
  // stable window against the Manual targets.
  private final ShotReadiness readiness = new ShotReadiness();
  private ShotReadiness.Evaluation lastEvaluation;

  public Manual() {
    addRequirements(
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
  }

  @Override
  public void initialize() {
    readiness.reset();
  }

  @Override
  public void execute() {
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);
    lastEvaluation =
        readiness.update(
            ManualConstants.ShooterRotpsTunable.getAsDouble(),
            ManualConstants.HoodDegTunable.getAsDouble() / 360.);

    // stableReady drops on the first below-speed sample (same-loop stop) and only turns true after
    // the full 0.15 s window, so the entry is debounced while the exit stays immediate.
    if (!lastEvaluation.stableReady()) {
      IntakeRoller.mInstance.setV(0);
      Indexer.mInstance.setV(0);
      Feeder.mInstance.setV(0);
      OperatorStatus.report(
          lastEvaluation.flywheelZero()
              ? OperatorStatus.Action.WAITING_RECOVERY
              : OperatorStatus.Action.SPINNING_UP);
      OperatorStatus.reportFeedDecision(
          false,
          lastEvaluation.flywheelZero()
              ? ShotReadiness.REASON_FLYWHEEL_ZERO
              : lastEvaluation.drumReady()
                  ? ShotReadiness.REASON_STABILIZING
                  : ShotReadiness.REASON_DRUM_NOT_AT_SPEED);
      return;
    }

    IntakeRoller.mInstance.setVelocityRotPerSec(
        ManualConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ManualConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ManualConstants.FeederRotpsTunable.getAsDouble());
    BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
    OperatorStatus.report(OperatorStatus.Action.FEEDING);
    OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    OperatorStatus.clearFeedDecision();
    Drum.mInstance.stop();
    Hood.mInstance.stop();
    IntakeRoller.mInstance.stop();
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
