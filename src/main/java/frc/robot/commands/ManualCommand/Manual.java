package frc.robot.commands.ManualCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.DashboardControls;
import frc.robot.util.OperatorStatus;
import frc.robot.util.ShotReadiness;

/**
 * X-button fixed-setpoint shot path. Every execute drives the drum and hood to the current Manual
 * targets and, whenever the flywheel limit is positive, immediately feeds at the Manual speeds.
 * Drum/hood readiness never pauses feeding; a zero flywheel limit stops only the feed path for that
 * loop, and restoring a positive limit resumes feeding on the next execute.
 */
public class Manual extends Command {
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
  public void execute() {
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);

    if (DashboardControls.getFlywheelScale() <= 0.0) {
      IntakeRoller.mInstance.setV(0);
      Indexer.mInstance.setV(0);
      Feeder.mInstance.setV(0);
      OperatorStatus.report(OperatorStatus.Action.WAITING_RECOVERY);
      OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_FLYWHEEL_ZERO);
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
