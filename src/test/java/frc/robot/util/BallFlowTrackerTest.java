package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.util.BallFlowTracker.FlowKind;
import frc.robot.util.BallFlowTracker.FlowState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The LED subsystem exposes the mechanism's semantic state as read-only diagnostics, so the
 * tracker's priority and conflict rules are pinned exactly: a clean direction is claimed only when
 * every fresh heartbeat agrees on it, and expiry (no heartbeat for 0.15 s) always falls back to
 * IDLE — no stuck flag can pin the diagnostic state forever. An injected clock keeps the timing
 * rules deterministic.
 */
class BallFlowTrackerTest {
  private final double[] now = {0.0};

  @BeforeEach
  void injectClock() {
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(() -> now[0]);
  }

  @AfterEach
  void resetTracker() {
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(null);
  }

  private void advanceTo(double t) {
    now[0] = t;
  }

  @Test
  void noReportsIsIdleAndSilent() {
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertEquals("", BallFlowTracker.diagnostic());
  }

  @Test
  void freshIntakeIsCleanForward() {
    BallFlowTracker.report(FlowKind.INTAKE);
    assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());
    assertEquals("", BallFlowTracker.diagnostic());
  }

  @Test
  void freshFeedIsCleanForward() {
    BallFlowTracker.report(FlowKind.FEED);
    assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());
    assertEquals("", BallFlowTracker.diagnostic());
  }

  @Test
  void feedAndIntakeTogetherAreStillCleanForward() {
    // Same direction from two sources is not a conflict.
    BallFlowTracker.report(FlowKind.INTAKE);
    BallFlowTracker.report(FlowKind.FEED);
    assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());
    assertEquals("", BallFlowTracker.diagnostic());
  }

  @Test
  void fullReverseAloneIsCleanReverse() {
    BallFlowTracker.report(FlowKind.FULL_REVERSE);
    assertEquals(FlowState.REVERSE, BallFlowTracker.effectiveState());
    assertEquals("", BallFlowTracker.diagnostic());
  }

  @Test
  void reverseFightingFeedIsIdleWithConflictNote() {
    BallFlowTracker.report(FlowKind.FULL_REVERSE);
    BallFlowTracker.report(FlowKind.FEED);
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertTrue(BallFlowTracker.diagnostic().contains("conflict"));
  }

  @Test
  void reverseFightingIntakeIsIdleWithConflictNote() {
    BallFlowTracker.report(FlowKind.FULL_REVERSE);
    BallFlowTracker.report(FlowKind.INTAKE);
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertTrue(BallFlowTracker.diagnostic().contains("conflict"));
  }

  @Test
  void localReverseAloneStaysIdleWithNote() {
    // UNJAM reverses only the feeder/indexer; it must never be reported as a whole-path reverse.
    BallFlowTracker.report(FlowKind.LOCAL_REVERSE);
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertTrue(BallFlowTracker.diagnostic().contains("local reverse"));
  }

  @Test
  void localReverseWithFeedStaysIdleWithNote() {
    BallFlowTracker.report(FlowKind.LOCAL_REVERSE);
    BallFlowTracker.report(FlowKind.FEED);
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertTrue(BallFlowTracker.diagnostic().contains("local reverse"));
  }

  @Test
  void localReverseWithFullReverseIsAConflict() {
    BallFlowTracker.report(FlowKind.LOCAL_REVERSE);
    BallFlowTracker.report(FlowKind.FULL_REVERSE);
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertTrue(BallFlowTracker.diagnostic().contains("conflict"));
  }

  @Test
  void reportsExpireAfter150ms() {
    BallFlowTracker.report(FlowKind.INTAKE);
    advanceTo(0.14);
    assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState(), "still within the expiry");
    advanceTo(0.16);
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState(), "heartbeat expired");
    assertEquals("", BallFlowTracker.diagnostic());
  }

  @Test
  void heartbeatKeepsTheReportAlive() {
    BallFlowTracker.report(FlowKind.INTAKE);
    advanceTo(0.10);
    BallFlowTracker.report(FlowKind.INTAKE);
    advanceTo(0.20);
    BallFlowTracker.report(FlowKind.INTAKE);
    advanceTo(0.34); // 0.14 s since the last heartbeat
    assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());
    advanceTo(0.36); // 0.16 s since the last heartbeat
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
  }

  @Test
  void conflictClearsAsSoonAsOneSideExpires() {
    BallFlowTracker.report(FlowKind.FULL_REVERSE);
    advanceTo(0.10);
    BallFlowTracker.report(FlowKind.FEED);

    advanceTo(0.14); // both reports fresh
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
    assertTrue(BallFlowTracker.diagnostic().contains("conflict"));

    advanceTo(0.16); // the reverse heartbeat (t=0) expired; feed (t=0.10) is still fresh
    assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());
    assertEquals("", BallFlowTracker.diagnostic());

    advanceTo(0.26); // the feed heartbeat expired too
    assertEquals(FlowState.IDLE, BallFlowTracker.effectiveState());
  }
}
