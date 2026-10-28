package frc.robot.commands.ManualCommand;

import edu.wpi.first.wpilibj.Timer;
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
import org.littletonrobotics.junction.Logger;

/**
 * Fixed-setpoint shot path with no physical controller binding; it remains directly schedulable.
 * Every execute drives the drum and hood to the current Manual targets. Initial feeding waits for
 * the measured Manual speed target to hold the 0.15 s stable window with a positive flywheel limit;
 * after that permit is earned, mid-volley speed sag and nonzero target edits do not interrupt
 * feeding. A 0% limit stops the feed path immediately and requires a fresh speed window on recovery
 * unless force-feed is active. The hood never gates feeding. The same force-feed timeout as
 * Shooting applies: waiting at a positive limit for the configured timeout (default 3 s,
 * Elastic-persisted via DashboardControls) latches the speed gate off for the rest of the command
 * run, so balls always come out.
 */
public class Manual extends Command {
  // Shared readiness rule with this command's own debounce, so each run re-earns the full stable
  // window against the Manual targets.
  private final ShotReadiness readiness = new ShotReadiness();
  private ShotReadiness.Evaluation lastEvaluation;
  // Force-feed timeout accumulator (positive-limit waiting only) and latch, reset per run.
  private final Timer gateWaitTimer = new Timer();
  private boolean forceFeedActive;
  // Normal mid-volley speed sag must not retract an already earned startup permit. A 0% edit does
  // revoke it, so recovery from the operator's deliberate stop must pass the speed window again.
  private boolean feedGateReleased;

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
    gateWaitTimer.stop();
    gateWaitTimer.reset();
    forceFeedActive = false;
    feedGateReleased = false;
  }

  @Override
  public void execute() {
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);
    lastEvaluation =
        readiness.update(
            ManualConstants.ShooterRotpsTunable.getAsDouble(),
            ManualConstants.HoodDegTunable.getAsDouble() / 360.);

    if (lastEvaluation.flywheelZero()) {
      feedGateReleased = false;
      gateWaitTimer.stop();
      IntakeRoller.mInstance.setV(0);
      Indexer.mInstance.setV(0);
      Feeder.mInstance.setV(0);
      OperatorStatus.report(OperatorStatus.Action.WAITING_RECOVERY);
      OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_FLYWHEEL_ZERO);
      return;
    }
    if (!forceFeedActive && !feedGateReleased && !lastEvaluation.stableReady()) {
      gateWaitTimer.start();
      if (gateWaitTimer.hasElapsed(DashboardControls.getForceFeedTimeoutSec())) {
        forceFeedActive = true;
      }
      if (!forceFeedActive) {
        IntakeRoller.mInstance.setV(0);
        Indexer.mInstance.setV(0);
        Feeder.mInstance.setV(0);
        OperatorStatus.report(OperatorStatus.Action.SPINNING_UP);
        OperatorStatus.reportFeedDecision(
            false,
            lastEvaluation.drumReady()
                ? ShotReadiness.REASON_STABILIZING
                : ShotReadiness.REASON_DRUM_NOT_AT_SPEED);
        return;
      }
    }

    feedGateReleased = true;
    // Feeding pauses the accumulated wait WITHOUT clearing it; only initialize() resets it.
    gateWaitTimer.stop();
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ManualConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ManualConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ManualConstants.FeederRotpsTunable.getAsDouble());
    BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
    OperatorStatus.report(OperatorStatus.Action.FEEDING);
    OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
    Logger.recordOutput("Manual/forceFeed", forceFeedActive);
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
