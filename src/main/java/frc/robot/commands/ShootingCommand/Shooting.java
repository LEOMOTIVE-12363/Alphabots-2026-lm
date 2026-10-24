package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.DashboardControls;
import frc.robot.util.OperatorStatus;
import frc.robot.util.ShotReadiness;
import org.littletonrobotics.junction.Logger;

/**
 * Demo shooting: drives the drum and hood from the Elastic Flywheel Speed Limit (%) and Hood Angle
 * (%) entries and feeds immediately whenever the flywheel limit is positive. No aiming, vision, or
 * pose/distance input is used; the drivetrain is not required.
 *
 * <p>The shared {@link ShotReadiness} rule is evaluated every loop only for genuine readiness
 * diagnostics. Measured drum speed, hood position, and the stable-ready debounce never permit or
 * inhibit feeding. Only a 0% flywheel limit pauses the feed path: feeder, indexer, and intake
 * roller stop on that loop while the drum and hood keep their targets. Any positive limit restores
 * feeding on that same loop. A 0% pause freezes the retract timer mid-count.
 *
 * <p>UNJAM always runs its back-off to completion, including at a 0% flywheel limit. Jam
 * confirmation accumulates only while feeding; PAUSED and UNJAM keep feeding the jam debouncer
 * clear samples so non-feeding time cannot pre-fill its window. The retract timer freezes when
 * UNJAM starts. When the back-off completes, a positive limit immediately restores the remembered
 * SHOOT or RETRACT outputs; a 0% limit enters PAUSED instead.
 *
 * <p>Releasing the key still ends the command outright: nothing auto-resumes after end(), and the
 * whileTrue binding re-initializes the command on the next press.
 */
public class Shooting extends Command {
  private enum States {
    SHOOT,
    RETRACT,
    PAUSED,
    UNJAM
  }

  private States state;

  // Times the delay between starting to feed and retracting the intake. Only 0% pauses and UNJAM
  // freeze this timer; measured readiness faults and target edits do not.
  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted;
  private final boolean isEnergySave;

  // Indexer jam detection: stator current must stay above the threshold for the full debounce
  // time before UNJAM triggers. Only feeding loops count; PAUSED and UNJAM feed clear samples.
  private Debouncer jamDebouncer;
  // Times the fixed backwards-run of the indexer in UNJAM.
  private final Timer unjamTimer = new Timer();
  // The feeding state (SHOOT or RETRACT) to resume once the unjam back-off finishes.
  private States stateAfterUnjam = States.SHOOT;
  // The feeding state (SHOOT or RETRACT) to resume once the flywheel limit becomes positive.
  private States stateAfterPause;

  // Real readiness measurements for the Shooting/*Ready log keys. These never gate feed outputs.
  private final ShotReadiness readiness = new ShotReadiness();
  private ShotReadiness.Evaluation lastEvaluation;

  public Shooting(boolean isEnergySave) {
    addRequirements(
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
    this.isEnergySave = isEnergySave;
  }

  @Override
  public void initialize() {
    state = States.SHOOT;
    retractTimerStarted = false;
    retractTimer.stop();
    retractTimer.reset();
    readiness.reset();
    resetJamDetection();
  }

  /** Hood target in mechanism rotations, from the Hood Angle (%) entry. */
  private double hoodTargetRot() {
    return DashboardControls.getHoodTargetDeg() / 360.;
  }

  /**
   * Drive the flywheel and hood to the dashboard setpoints. The drum is asked for full shot speed;
   * Drum applies Flywheel Speed Limit (%) to it, so the percentage is applied exactly once.
   */
  private void prepareShooter() {
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.MaxShotVelocity);
    Hood.mInstance.setPositionRot(hoodTargetRot());
    Logger.recordOutput(
        "Shooting/drumSetpointRotps",
        DashboardControls.applyFlywheelLimit(DrumConstants.MaxShotVelocity));
    Logger.recordOutput("Shooting/hoodSetpointDeg", DashboardControls.getHoodTargetDeg());
  }

  /** Update real readiness diagnostics once per execute; the result never gates feeding. */
  private ShotReadiness.Evaluation updateReadiness() {
    lastEvaluation = readiness.update(DrumConstants.MaxShotVelocity, hoodTargetRot());
    Logger.recordOutput("Shooting/drumReady", lastEvaluation.drumReady());
    Logger.recordOutput("Shooting/hoodReady", lastEvaluation.hoodReady());
    return lastEvaluation;
  }

  /**
   * Start jam detection from a clean baseline. Only initialize() needs a fresh debouncer; the
   * non-feeding states keep the running one re-anchored via holdJamNotDetected().
   */
  private void resetJamDetection() {
    jamDebouncer = new Debouncer(ShootingConstants.INDEXER_JAM_DEBOUNCE_SEC, DebounceType.kRising);
  }

  /**
   * Feed the jam debouncer a clear sample without reading the mechanism. A kRising debouncer
   * re-anchors its timer only on baseline (clear) samples and lets it free-run otherwise, so every
   * non-feeding state must call this each loop: otherwise pause or back-off time would count toward
   * the confirmation window, and the first high-current sample after feeding resumed would trip
   * UNJAM instantly instead of having to persist for the full debounce.
   */
  private void holdJamNotDetected() {
    jamDebouncer.calculate(false);
  }

  /** True when the indexer stator current has been above the jam threshold for the debounce. */
  private boolean isIndexerJammed() {
    boolean jammed =
        jamDebouncer.calculate(
            Indexer.mInstance.getStatorCurrentAmps() > ShootingConstants.INDEXER_JAM_CURRENT_AMPS);
    Logger.recordOutput("Shooting/indexerJammed", jammed);
    return jammed;
  }

  /** Reverse only the feeder/indexer while holding the shooter targets during UNJAM. */
  private void writeUnjamOutputs() {
    prepareShooter();
    IntakeRoller.mInstance.setV(0);
    Feeder.mInstance.setV(-ShootingConstants.UNJAM_Feeder_Current.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(-ShootingConstants.UNJAM_INDEXER_Current.getAsDouble());
    IntakeDeploy.mInstance.setPositionCentimeter(55);
  }

  /**
   * SHOOT/RETRACT -> UNJAM when the indexer stalls. The reverse outputs and retract freeze start on
   * the detection loop, and the interrupted feeding state is remembered for immediate recovery.
   */
  private boolean checkJamAndStartUnjam(States resumeState) {
    if (!isIndexerJammed()) {
      return false;
    }
    stateAfterUnjam = resumeState;
    if (retractTimerStarted) {
      retractTimer.stop();
    }
    state = States.UNJAM;
    unjamTimer.restart();
    writeUnjamOutputs();
    return true;
  }

  /** Run the fixed back-off, then immediately resume feeding if the flywheel limit is positive. */
  private void unjam() {
    // Back-off time must not pre-fill the next jam confirmation window.
    holdJamNotDetected();
    if (!unjamTimer.hasElapsed(ShootingConstants.UNJAM_DURATION_SEC.getAsDouble())) {
      writeUnjamOutputs();
      return;
    }

    if (lastEvaluation.flywheelZero()) {
      enterPause(stateAfterUnjam);
      holdPauseOutputs();
      return;
    }

    state = stateAfterUnjam;
    if (state == States.RETRACT) {
      retract();
    } else {
      shoot();
    }
    // Feeding resumes now, but the completion execute remains in UNJAM's frozen retract interval.
    // startOrResumeRetractTimer() picks the accumulated count back up on the next feeding loop.
    retractTimer.stop();
  }

  /** Drive the full feed path (intake roller, indexer, feeder) at the tuned shooting speeds. */
  private void runFeed() {
    if (IntakeDeploy.mInstance.getPositionCentimeter() <= 30) {
      IntakeRoller.mInstance.setVelocityRotPerSec(0);
    } else {
      IntakeRoller.mInstance.setVelocityRotPerSec(
          ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    }
    if (!isEnergySave) {
      Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
      Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
    } else {
      Indexer.mInstance.setCurrent(50);
      Feeder.mInstance.setCurrent(50);
    }
  }

  /** Start the retract delay on the first feed and resume it after a 0% pause or UNJAM. */
  private void startOrResumeRetractTimer() {
    if (!retractTimerStarted) {
      retractTimer.restart();
      retractTimerStarted = true;
    } else {
      retractTimer.start();
    }
  }

  private void shoot() {
    if (checkJamAndStartUnjam(States.SHOOT)) {
      return;
    }
    if (lastEvaluation.flywheelZero()) {
      enterPause(States.SHOOT);
      holdPauseOutputs();
      holdJamNotDetected();
      return;
    }

    prepareShooter();
    runFeed();
    startOrResumeRetractTimer();
    if (retractTimer.hasElapsed(ShootingConstants.RETRACT_DELAY_SEC.getAsDouble())) {
      state = States.RETRACT;
    }
  }

  private void retract() {
    if (checkJamAndStartUnjam(States.RETRACT)) {
      return;
    }
    if (lastEvaluation.flywheelZero()) {
      enterPause(States.RETRACT);
      holdPauseOutputs();
      holdJamNotDetected();
      return;
    }

    prepareShooter();
    runFeed();
    startOrResumeRetractTimer();
    IntakeDeploy.mInstance.setPositionCentimeter(
        ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
  }

  /** Remember the feeding state and freeze retract timing for a 0% flywheel pause. */
  private void enterPause(States resume) {
    stateAfterPause = resume;
    if (retractTimerStarted) {
      retractTimer.stop();
    }
    state = States.PAUSED;
  }

  /** Stop only the feed path at 0%; the drum/hood and any active retract keep their targets. */
  private void holdPauseOutputs() {
    prepareShooter();
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
    if (stateAfterPause == States.RETRACT) {
      IntakeDeploy.mInstance.setPositionCentimeter(
          ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
    }
  }

  /** Hold at 0%, or restore the remembered feeding outputs on the first positive-limit loop. */
  private void paused() {
    if (lastEvaluation.flywheelZero()) {
      holdPauseOutputs();
      holdJamNotDetected();
      return;
    }

    state = stateAfterPause;
    if (state == States.RETRACT) {
      retract();
    } else {
      shoot();
    }
  }

  @Override
  public void execute() {
    updateReadiness();
    switch (state) {
      case SHOOT -> shoot();
      case RETRACT -> retract();
      case PAUSED -> paused();
      case UNJAM -> unjam();
    }
    // PAUSED reports no ball flow; UNJAM is a local feeder/indexer reverse, never a full-path one.
    switch (state) {
      case SHOOT, RETRACT -> BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
      case UNJAM -> BallFlowTracker.report(BallFlowTracker.FlowKind.LOCAL_REVERSE);
      case PAUSED -> {}
    }
    // Report the action produced by this execute, including immediate pause/recovery transitions.
    switch (state) {
      case SHOOT, RETRACT -> OperatorStatus.report(OperatorStatus.Action.FEEDING);
      case PAUSED -> OperatorStatus.report(OperatorStatus.Action.WAITING_RECOVERY);
      case UNJAM -> OperatorStatus.report(OperatorStatus.Action.CLEARING_JAM);
    }
    // The panel decision follows the actual outputs, never the measured readiness diagnostics.
    switch (state) {
      case SHOOT, RETRACT -> OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
      case PAUSED -> OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_FLYWHEEL_ZERO);
      case UNJAM -> OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_CLEARING_JAM);
    }
    Logger.recordOutput("Shooting/state", state.toString());
    Logger.recordOutput("Shooting/stableReady", lastEvaluation.stableReady());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    OperatorStatus.clearFeedDecision();
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.StowVelocity);
    Hood.mInstance.setPositionRot(0);
    IntakeRoller.mInstance.stop();
    IntakeDeploy.mInstance.setPositionCentimeter(IntakeDeployConstants.IntakeOutPosition);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
