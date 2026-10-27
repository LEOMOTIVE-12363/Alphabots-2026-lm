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
 * Demo shooting: spins the drum and sets the hood from the Elastic Flywheel Speed Limit (%) and
 * Hood Angle (%) entries, then gates initial feeding on measured flywheel speed. No aiming, vision,
 * or pose/distance input is used; the drivetrain is not required.
 *
 * <p>The startup gate uses the shared {@link ShotReadiness} rule: drum within {@link
 * ShootingConstants#SHOOTER_VELOCITY_TOLERANCE_ROTPS} of the limited target, held for the {@link
 * ShootingConstants#READY_DEBOUNCE_SEC} stable window. Once that permit is earned, ordinary
 * mid-volley speed sag and nonzero target edits do not interrupt feeding, avoiding a load-induced
 * pause/recovery cycle on every ball. A 0% limit still stops the feed path immediately and revokes
 * the permit, so a positive value must earn the window again unless force-feed is active. The hood
 * never gates feeding — it stays a logged diagnostic. Pauses for 0% and UNJAM freeze the retract
 * timer mid-count.
 *
 * <p>Force-feed timeout: if the gate still has not released feeding after {@link
 * DashboardControls#getForceFeedTimeoutSec} (default 3 s, Elastic-persisted) of accumulated waiting
 * (spin-up plus below-speed pauses; a 0% limit never counts), the volley force-feeds for the rest
 * of the press — the speed gate latches off until the button is released, so balls always come out
 * even when the drum can never reach its target (a pure-P velocity loop leaves a standing error
 * under load).
 *
 * <p>UNJAM always runs its back-off to completion regardless of readiness — clearing a jam is never
 * suppressed by the not-ready gate — then routes through PAUSED. Forward feeding normally re-earns
 * a full stable window because the readiness debounce resets at jam entry; an already-active
 * force-feed latch instead resumes directly after the back-off. The jam confirmation window only
 * accumulates while feeding: every non-feeding state (SPIN_UP, PAUSED, UNJAM) keeps feeding the jam
 * debouncer clear samples, so time spent spinning up, paused, or backing out a jam never pre-fills
 * the window.
 *
 * <p>Releasing the key still ends the command outright: nothing auto-resumes after end(), and the
 * whileTrue binding re-initializes the command (and the readiness debounce) on the next press.
 */
public class Shooting extends Command {
  private enum States {
    SPIN_UP,
    SHOOT,
    RETRACT,
    PAUSED,
    UNJAM
  }

  private States state;

  // Times the delay between starting to feed and retracting the intake. SPIN_UP and PAUSED never
  // run it; 0% pauses and UNJAM freeze it mid-count and only feeding loops start or resume it.
  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted;
  private final boolean isEnergySave;

  // Indexer jam detection: stator current must stay above the threshold for the full debounce
  // time before UNJAM triggers. Only feeding loops count; SPIN_UP, PAUSED, and UNJAM feed clear
  // samples.
  private Debouncer jamDebouncer;
  // Times the fixed backwards-run of the indexer in UNJAM.
  private final Timer unjamTimer = new Timer();
  // The feeding state (SHOOT or RETRACT) to resume once the unjam back-off finishes.
  private States stateAfterUnjam = States.SHOOT;
  // The feeding state (SHOOT or RETRACT) PAUSED returns to once the shot is stable-ready again.
  private States stateAfterPause;

  // Shared readiness rule (the flywheel-speed gate plus the hood diagnostic), with this command's
  // own debounce state; lastEvaluation backs the Shooting/*Ready log keys and the panel decision.
  private final ShotReadiness readiness = new ShotReadiness();
  private ShotReadiness.Evaluation lastEvaluation;

  // Force-feed timeout: accumulates only while the gate is actually holding feeding back at a
  // positive limit (SPIN_UP and below-speed PAUSED; a 0% limit never counts). Once the wait
  // exceeds the configured timeout the volley force-feeds for the rest of the press — the speed
  // gate latches off so balls always come out — until the button is released and re-pressed.
  private final Timer gateWaitTimer = new Timer();
  private boolean forceFeedActive;
  // Whether this startup cycle has earned feed permission. Normal mid-volley speed sag does not
  // revoke it; 0% and UNJAM do, so those recovery paths must earn it again unless force-feed is on.
  private boolean feedGateReleased;

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
    state = States.SPIN_UP;
    retractTimerStarted = false;
    retractTimer.stop();
    retractTimer.reset();
    gateWaitTimer.stop();
    gateWaitTimer.reset();
    forceFeedActive = false;
    feedGateReleased = false;
    readiness.reset();
    resetJamDetection();
  }

  /**
   * Tracks the accumulated gate wait for the current non-feeding loop: counting only at a positive
   * limit (a 0% limit is the operator deliberately holding the drum stopped and never times into
   * feeding), and latching force-feed once the timeout elapses.
   */
  private void trackGateWait(boolean gateHoldingAtPositiveLimit) {
    if (gateHoldingAtPositiveLimit) {
      gateWaitTimer.start();
      if (gateWaitTimer.hasElapsed(DashboardControls.getForceFeedTimeoutSec())) {
        forceFeedActive = true;
      }
    } else {
      gateWaitTimer.stop();
    }
  }

  /**
   * Pauses the wait accumulator WITHOUT clearing it: a brief feeding stretch, 0% hold, or UNJAM
   * must not erase waiting already accumulated during startup or an earlier recovery. Only
   * initialize() resets the accumulator (a fresh press).
   */
  private void pauseGateWait() {
    gateWaitTimer.stop();
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

  /**
   * One readiness evaluation for this loop against the dashboard shot configuration, logging the
   * readiness keys the operator panel consumes. Every state except UNJAM calls this exactly once
   * per loop; UNJAM deliberately holds not-ready samples instead — a jam back-off must never be
   * suppressed by the not-ready gate, and its quiet time must not let the stable debounce free-run.
   */
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
   * non-feeding state must call this each loop: otherwise time spent spinning up, paused, or
   * backing out a jam would silently count toward the confirmation window, and the first
   * high-current sample after feeding resumed would trip UNJAM instantly instead of having to
   * persist for the full debounce.
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

  /**
   * SHOOT/RETRACT -> UNJAM when the indexer stalls. Returns true when the transition fired;
   * remembers the interrupted state so the volley resumes exactly where it left off (through
   * PAUSED, see unjam()). The retract timer freezes the moment the back-off starts and the
   * readiness debounce resets, so after the back-off the shot must re-earn a fresh stable-ready
   * window before feeding resumes.
   */
  private boolean checkJamAndStartUnjam(States resumeState) {
    if (!isIndexerJammed()) {
      return false;
    }
    stateAfterUnjam = resumeState;
    if (retractTimerStarted) {
      retractTimer.stop();
    }
    readiness.reset();
    feedGateReleased = false;
    state = States.UNJAM;
    unjamTimer.restart();
    writeUnjamOutputs();
    return true;
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
   * Back the stalled fuel out of the indexer, then re-gate through PAUSED: forward feeding normally
   * resumes only once the flywheel is stable-ready again, or immediately when force-feed is already
   * active. The back-off itself always completes regardless of readiness.
   */
  private void unjam() {
    // Back-off time must not pre-fill the next jam confirmation window, and it is not gate
    // waiting either: pause (not clear) the accumulated force-feed wait.
    holdJamNotDetected();
    pauseGateWait();
    if (!unjamTimer.hasElapsed(ShootingConstants.UNJAM_DURATION_SEC.getAsDouble())) {
      writeUnjamOutputs();
      return;
    }

    enterPause(stateAfterUnjam);
    holdPauseOutputs();
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
    // A 0% limit always pauses, even after the force-feed timeout: the operator is deliberately
    // holding the drum stopped.
    if (lastEvaluation.flywheelZero()) {
      feedGateReleased = false;
      enterPause(States.SHOOT);
      holdPauseOutputs();
      holdJamNotDetected();
      return;
    }

    pauseGateWait();
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
      feedGateReleased = false;
      enterPause(States.RETRACT);
      holdPauseOutputs();
      holdJamNotDetected();
      return;
    }

    pauseGateWait();
    prepareShooter();
    runFeed();
    startOrResumeRetractTimer();
    IntakeDeploy.mInstance.setPositionCentimeter(
        ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
  }

  /**
   * Freeze the volley exactly where it is: remember the feeding state to resume and freeze the
   * retract timer mid-count (Timer.start() on resume picks the accumulated count back up, so paused
   * time never counts toward the retract delay).
   */
  private void enterPause(States resume) {
    stateAfterPause = resume;
    if (retractTimerStarted) {
      retractTimer.stop();
    }
    state = States.PAUSED;
  }

  /** Stop only the feed path; the drum/hood and any active retract keep their targets. */
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

  /** Hold at 0% or while a recovery path must re-earn the startup gate. */
  private void paused() {
    if (lastEvaluation.flywheelZero()) {
      feedGateReleased = false;
      trackGateWait(false); // a 0% limit never accumulates toward the force-feed timeout
      holdPauseOutputs();
      holdJamNotDetected();
      return;
    }
    if (!forceFeedActive && !feedGateReleased && !lastEvaluation.stableReady()) {
      // A recovery path still waiting at a positive limit accumulates toward the same timeout.
      trackGateWait(true);
      holdPauseOutputs();
      holdJamNotDetected();
      if (forceFeedActive) {
        feedGateReleased = true;
        resumeFromPause();
      }
      return;
    }

    feedGateReleased = true;
    resumeFromPause();
  }

  /** Restore the remembered feeding state from PAUSED on this loop. */
  private void resumeFromPause() {
    pauseGateWait();
    state = stateAfterPause;
    if (state == States.RETRACT) {
      retract();
    } else {
      shoot();
    }
  }

  /** Hold the shooter on target with the feed path stopped until the flywheel is stable-ready. */
  private void spinUp() {
    prepareShooter();
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
    // Spin-up is not feeding: keep the jam debouncer anchored, or a long real spin-up would
    // pre-fill the confirmation window and the first high-current sample would trip UNJAM.
    holdJamNotDetected();

    if (lastEvaluation.stableReady()) {
      feedGateReleased = true;
      state = States.SHOOT;
      return;
    }
    // Waiting at a positive limit accumulates toward the force-feed timeout (0% does not); once
    // it elapses the volley starts feeding even though the flywheel never reached its target.
    trackGateWait(!lastEvaluation.flywheelZero());
    if (forceFeedActive) {
      feedGateReleased = true;
      state = States.SHOOT;
    }
  }

  /** The blocked reason for a non-feeding, non-unjam loop. */
  private String pausedReason() {
    if (lastEvaluation.flywheelZero()) {
      return ShotReadiness.REASON_FLYWHEEL_ZERO;
    }
    return lastEvaluation.drumReady()
        ? ShotReadiness.REASON_STABILIZING // at speed but inside the debounce window
        : ShotReadiness.REASON_DRUM_NOT_AT_SPEED;
  }

  private OperatorStatus.Action pausedAction() {
    return lastEvaluation.flywheelZero()
        ? OperatorStatus.Action.WAITING_RECOVERY
        : OperatorStatus.Action.SPINNING_UP;
  }

  @Override
  public void execute() {
    // UNJAM never evaluates readiness: the back-off must not be suppressed by the not-ready gate,
    // and its quiet time must not let the stable debounce free-run.
    if (state == States.UNJAM) {
      readiness.holdNotReady();
    } else {
      updateReadiness();
    }
    switch (state) {
      case SPIN_UP -> spinUp();
      case SHOOT -> shoot();
      case RETRACT -> retract();
      case PAUSED -> paused();
      case UNJAM -> unjam();
    }
    // SPIN_UP and PAUSED report no ball flow; UNJAM is a local feeder/indexer reverse.
    switch (state) {
      case SHOOT, RETRACT -> BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
      case UNJAM -> BallFlowTracker.report(BallFlowTracker.FlowKind.LOCAL_REVERSE);
      case SPIN_UP, PAUSED -> {}
    }
    // Report the action produced by this execute, including immediate pause/recovery transitions.
    switch (state) {
      case SHOOT, RETRACT -> OperatorStatus.report(OperatorStatus.Action.FEEDING);
      case SPIN_UP -> OperatorStatus.report(OperatorStatus.Action.SPINNING_UP);
      case PAUSED -> OperatorStatus.report(pausedAction());
      case UNJAM -> OperatorStatus.report(OperatorStatus.Action.CLEARING_JAM);
    }
    // The panel decision follows the actual outputs, never the hood diagnostic.
    switch (state) {
      case SHOOT, RETRACT -> OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
      case SPIN_UP, PAUSED -> OperatorStatus.reportFeedDecision(false, pausedReason());
      case UNJAM -> OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_CLEARING_JAM);
    }
    Logger.recordOutput("Shooting/state", state.toString());
    Logger.recordOutput("Shooting/stableReady", lastEvaluation.stableReady());
    Logger.recordOutput("Shooting/forceFeed", forceFeedActive);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    OperatorStatus.clearFeedDecision();
    // Stop, not stow: no standing velocity request may remain, or re-enabling the robot would
    // spin the drum back up with nobody touching the controls.
    Drum.mInstance.stop();
    Hood.mInstance.setPositionRot(0);
    IntakeRoller.mInstance.stop();
    IntakeDeploy.mInstance.setPositionCentimeter(IntakeDeployConstants.IntakeOutPosition);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
