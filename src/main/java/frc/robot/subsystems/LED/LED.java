package frc.robot.subsystems.LED;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.BallFlowTracker.FlowState;
import frc.robot.util.DashboardControls;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Addressable LED strip. The Elastic "LED Pattern" chooser picks between the five legacy effects
 * and the five Day of the Dead themes; theme patterns render the theme gradient with the
 * independently chosen "LED Idle Effect". The selected style is independent of enable state and
 * mechanism activity. Global brightness comes from the "/Tuning/LED Brightness (%)" input. Pattern,
 * idle effect, and brightness are power-cycle persistent like the other Elastic settings.
 *
 * <p>All timing derives from the AdvantageKit log timestamp so replays reproduce the exact output;
 * in replay mode nothing is written to NetworkTables or the settings file.
 */
public class LED extends SubsystemBase {
  public static final LED mInstance = new LED();

  /** Settings-store key of the pattern chooser; its NT entry is "/SmartDashboard/LED Pattern". */
  public static final String PATTERN_SETTING_KEY = "/SmartDashboard/LED Pattern";

  /** Settings-store key of the idle-effect chooser; NT entry "/SmartDashboard/LED Idle Effect". */
  public static final String EFFECT_SETTING_KEY = "/SmartDashboard/LED Idle Effect";

  private static final int MAX_VALUE = 255;

  /**
   * Loops after boot during which no idle-effect recommendation is written: the restore writes to
   * the choosers' "selected" entries need a few loops to propagate through NT, and until they do
   * the restored theme must not be mistaken for an operator change (which would overwrite the
   * restored effect with the theme's recommendation).
   */
  private static final int BOOT_RECOMMENDATION_SUPPRESSION_LOOPS = 5;

  private final LoggedDashboardChooser<LEDPattern> patternChooser =
      new LoggedDashboardChooser<>("LED Pattern");
  private final LoggedDashboardChooser<IdleEffect> effectChooser =
      new LoggedDashboardChooser<>("LED Idle Effect");

  private final LEDIO io;
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);
  private final LEDLayout layout = LEDLayout.fromConstants();
  private final LEDSection[] sections = layout.sections();
  private final LEDPreview preview = new LEDPreview();

  /**
   * The CANdle onboard thumbnail: the shared render logic evaluated at eight normalized ball-path
   * positions (a single non-reversed BALL_PATH section, so onboard 0→7 is always intake→shooter —
   * independent of how the external strip happens to be wired). Ignored by the PWM driver and in
   * replay.
   */
  private final AddressableLEDBuffer onboardBuffer =
      new AddressableLEDBuffer(LEDConstants.ONBOARD_THUMBNAIL_PIXELS);

  private static final LEDSection[] ONBOARD_LOGICAL_SECTIONS = {
    new LEDSection(0, LEDConstants.ONBOARD_THUMBNAIL_PIXELS, false, LEDSection.Role.BALL_PATH)
  };

  private LEDPattern lastPattern; // restore intent; baseline for spotting operator theme changes
  private int loopsSinceBoot;
  private LEDTheme activeTheme;
  private LEDTheme previousTheme;
  private double themeTransitionStartSec;

  private LED() {
    io = LEDIO.create();

    patternChooser.addOption(LEDPattern.OFF.getDashboardName(), LEDPattern.OFF);
    patternChooser.addDefaultOption(
        LEDPattern.MARIGOLD_BRIDGE.getDashboardName(), LEDPattern.MARIGOLD_BRIDGE);
    for (LEDPattern pattern : LEDPattern.values()) {
      if (pattern != LEDPattern.OFF && pattern != LEDPattern.MARIGOLD_BRIDGE) {
        patternChooser.addOption(pattern.getDashboardName(), pattern);
      }
    }
    effectChooser.addDefaultOption(IdleEffect.SLOW_FLOW.getDashboardName(), IdleEffect.SLOW_FLOW);
    for (IdleEffect effect : IdleEffect.values()) {
      if (effect != IdleEffect.SLOW_FLOW) {
        effectChooser.addOption(effect.getDashboardName(), effect);
      }
    }

    restoreSavedPattern();
    restoreSavedEffect();
    activeTheme = lastPattern != null ? lastPattern.theme().orElse(null) : null;
  }

  /**
   * Re-selects the pattern saved by the previous run, exactly as if the operator had picked it on
   * the dashboard (writes the chooser's "selected" NT entry, which Elastic drives the same way),
   * and establishes the baseline used to spot operator theme changes later. An unknown saved name
   * (e.g. left over from an older build) is ignored, keeping the Marigold Bridge default.
   */
  private void restoreSavedPattern() {
    if (Logger.hasReplaySource()) {
      // No restore intent in replay; the first periodic adopts whatever the log replays.
      lastPattern = null;
      return;
    }
    lastPattern = LEDPattern.MARIGOLD_BRIDGE;
    String saved = DashboardControls.getSavedString(PATTERN_SETTING_KEY, null);
    if (saved == null) {
      return;
    }
    for (LEDPattern pattern : LEDPattern.values()) {
      if (pattern.getDashboardName().equals(saved)) {
        selectOnChooser(PATTERN_SETTING_KEY, saved);
        lastPattern = pattern;
        return;
      }
    }
    System.err.println("[LED] Unknown saved pattern \"" + saved + "\"; keeping the default");
  }

  /**
   * Restores the saved idle effect like {@link #restoreSavedPattern()}. An unknown saved name falls
   * back to the restored theme's recommended effect — written to the chooser so the dashboard shows
   * what actually renders; a restored legacy pattern (no theme) keeps the chooser default.
   */
  private void restoreSavedEffect() {
    if (Logger.hasReplaySource()) {
      return;
    }
    String saved = DashboardControls.getSavedString(EFFECT_SETTING_KEY, null);
    if (saved == null) {
      return;
    }
    if (IdleEffect.fromDashboardName(saved, null) != null) {
      selectOnChooser(EFFECT_SETTING_KEY, saved);
      return;
    }
    System.err.println(
        "[LED] Unknown saved idle effect \"" + saved + "\"; using the theme's recommended effect");
    if (lastPattern != null) {
      lastPattern
          .theme()
          .ifPresent(
              theme ->
                  selectOnChooser(
                      EFFECT_SETTING_KEY, theme.getRecommendedEffect().getDashboardName()));
    }
  }

  /**
   * Advances through the five themes from the chooser's current value; legacy patterns start at
   * Marigold.
   */
  public void cycleTheme() {
    if (Logger.hasReplaySource()) {
      return;
    }
    LEDPattern[] cycle = {
      LEDPattern.MARIGOLD_BRIDGE,
      LEDPattern.FESTIVAL_NIGHT,
      LEDPattern.PAPEL_FIESTA,
      LEDPattern.ALEBRIJE_GLOW,
      LEDPattern.CANDLE_MEMORY
    };
    LEDPattern current = patternChooser.get();
    int next = 0;
    for (int i = 0; i < cycle.length; i++) {
      if (current == cycle[i]) {
        next = (i + 1) % cycle.length;
        break;
      }
    }
    selectOnChooser(PATTERN_SETTING_KEY, cycle[next].getDashboardName());
  }

  /**
   * Advances the idle effect only while a theme is selected; legacy patterns leave both choosers
   * alone.
   */
  public void cycleIdleEffect() {
    if (Logger.hasReplaySource()) {
      return;
    }
    LEDPattern pattern = patternChooser.get();
    if (pattern == null || pattern.theme().isEmpty()) {
      return;
    }
    IdleEffect[] cycle = IdleEffect.values();
    IdleEffect current = effectChooser.get();
    int next = 0;
    for (int i = 0; i < cycle.length; i++) {
      if (current == cycle[i]) {
        next = (i + 1) % cycle.length;
        break;
      }
    }
    selectOnChooser(EFFECT_SETTING_KEY, cycle[next].getDashboardName());
  }

  /** Writes a chooser's "selected" NT entry, the same way Elastic applies an operator pick. */
  private static void selectOnChooser(String settingKey, String dashboardName) {
    NetworkTableInstance.getDefault().getEntry(settingKey + "/selected").setString(dashboardName);
  }

  @Override
  public void periodic() {
    double now = Logger.getTimestamp() / 1e6;
    loopsSinceBoot++;

    LEDPattern pattern = patternChooser.get();
    if (pattern == null) {
      pattern = LEDPattern.OFF;
    }
    recommendEffectOnThemeChange(pattern);

    IdleEffect effect = effectChooser.get();
    if (effect == null) {
      effect = IdleEffect.SLOW_FLOW;
    }
    double brightnessPercent = DashboardControls.getLedBrightnessPercent();

    DashboardControls.persistStringIfChanged(PATTERN_SETTING_KEY, pattern.getDashboardName());
    DashboardControls.persistStringIfChanged(EFFECT_SETTING_KEY, effect.getDashboardName());

    // Tracker state is retained for read-only diagnostics only; it never feeds the render path.
    boolean disabled = DriverStation.isDisabled();
    FlowState flowTarget = disabled ? FlowState.IDLE : BallFlowTracker.effectiveState();

    if (pattern.theme().isPresent()) {
      renderThemePattern(pattern.theme().get(), effect, now, brightnessPercent);
    } else {
      renderLegacyPattern(pattern, now, brightnessPercent);
    }
    // Onboard first so the driver plans with the current thumbnail when it sends the external
    // frame in the same call chain.
    io.writeOnboardBuffer(onboardBuffer);
    io.writeBuffer(buffer);

    publishStatus(pattern, effect, brightnessPercent, disabled, flowTarget);
    preview.update(pattern, effect, now);
  }

  /**
   * When the operator switches the pattern to a theme, pre-selects that theme's recommended idle
   * effect (the operator can override it afterwards). Suppressed for the first loops after boot so
   * a restored theme+effect pair survives: the boot restore never counts as an operator change.
   */
  private void recommendEffectOnThemeChange(LEDPattern pattern) {
    if (lastPattern == null) {
      // No restore ran (replay): adopt the first observation as the baseline.
      lastPattern = pattern;
      return;
    }
    if (loopsSinceBoot <= BOOT_RECOMMENDATION_SUPPRESSION_LOOPS) {
      return;
    }
    if (pattern == lastPattern) {
      return;
    }
    lastPattern = pattern;
    if (Logger.hasReplaySource()) {
      return;
    }
    pattern
        .theme()
        .ifPresent(
            theme ->
                selectOnChooser(
                    EFFECT_SETTING_KEY, theme.getRecommendedEffect().getDashboardName()));
  }

  /**
   * Renders a Day of the Dead theme into the external strip's physical sections and into the
   * onboard thumbnail's logical ball-path section — one shared code path, so theme, clock,
   * animation phase and brightness are identical on both (the onboard direction is fixed by its
   * non-reversed logical section and never depends on the external wiring).
   */
  private void renderThemePattern(
      LEDTheme theme, IdleEffect effect, double now, double brightnessPercent) {
    Function<Double, Color> themeColor = transitioningThemeColor(theme, now);
    renderThemeInto(buffer, sections, themeColor, effect, now, brightnessScale(brightnessPercent));
    renderThemeInto(
        onboardBuffer,
        ONBOARD_LOGICAL_SECTIONS,
        themeColor,
        effect,
        now,
        brightnessScale(brightnessPercent));
  }

  private static double brightnessScale(double brightnessPercent) {
    return brightnessPercent / 100.0;
  }

  /**
   * The shared theme render: idle effect (gradient × temporal envelope) per section over its
   * logical u, then global brightness as one final pass. The result is logical RGB; the physical
   * strip channel order is applied by the IO driver. Package-private static so tests can pin the
   * onboard logical rendering directly.
   */
  static void renderThemeInto(
      AddressableLEDBuffer target,
      LEDSection[] targetSections,
      Function<Double, Color> themeColor,
      IdleEffect effect,
      double now,
      double brightnessScale) {
    int length = target.getLength();
    if (brightnessScale <= 0.0) {
      for (int i = 0; i < length; i++) {
        target.setRGB(i, 0, 0, 0);
      }
      return;
    }
    // Pixels not covered by any section would otherwise keep stale colors from a previous frame.
    for (int i = 0; i < length; i++) {
      target.setRGB(i, 0, 0, 0);
    }
    for (LEDSection section : targetSections) {
      int start = Math.max(section.startPixel(), 0);
      int end = Math.min(section.startPixel() + section.lengthPixels(), length);
      for (int i = start; i < end; i++) {
        double u = section.uForPixel(i);
        Color color =
            IdleEffects.scale(
                IdleEffects.colorAt(effect, themeColor, u, now),
                IdleEffects.envelope(effect, u, now));
        target.setRGB(
            i,
            IdleEffects.to255(color.red),
            IdleEffects.to255(color.green),
            IdleEffects.to255(color.blue));
      }
    }
    // Global brightness applies once, at the very end. Skipped entirely at full brightness so the
    // 8-bit round trip cannot round a pixel down by one count. Physical color-order conversion
    // (WS2811 R/G swap) happens once more, inside each IO driver — see LEDIO.
    if (brightnessScale < 1.0) {
      for (int i = 0; i < length; i++) {
        Color color = IdleEffects.scale(target.getLED(i), brightnessScale);
        target.setRGB(
            i,
            IdleEffects.to255(color.red),
            IdleEffects.to255(color.green),
            IdleEffects.to255(color.blue));
      }
    }
  }

  /**
   * The active theme's gradient, RGB-crossfaded from the previous theme for {@link
   * LEDConstants#THEME_TRANSITION_SEC} after an operator theme switch (boot starts directly on the
   * restored theme, without a transition).
   */
  private Function<Double, Color> transitioningThemeColor(LEDTheme theme, double now) {
    if (theme != activeTheme) {
      previousTheme = activeTheme;
      activeTheme = theme;
      themeTransitionStartSec = now;
    }
    if (previousTheme == null) {
      return activeTheme::colorAt;
    }
    double blend = (now - themeTransitionStartSec) / LEDConstants.THEME_TRANSITION_SEC;
    if (blend >= 1.0) {
      previousTheme = null;
      return activeTheme::colorAt;
    }
    LEDTheme from = previousTheme;
    LEDTheme to = activeTheme;
    return u -> IdleEffects.lerp(from.colorAt(u), to.colorAt(u), blend);
  }

  /**
   * The five legacy effects with their original per-pattern math (including brightness applied to
   * the HSV value channel); they use their own pattern math rather than theme sections. Rendered
   * into the external buffer and the onboard thumbnail with the same math — the onboard shows the
   * same pattern at 8-pixel granularity.
   */
  private void renderLegacyPattern(LEDPattern pattern, double now, double brightnessPercent) {
    renderLegacyInto(buffer, pattern, now, brightnessPercent);
    renderLegacyInto(onboardBuffer, pattern, now, brightnessPercent);
  }

  /** Package-private static sibling of {@link #renderLegacyPattern} for direct testing. */
  static void renderLegacyInto(
      AddressableLEDBuffer target, LEDPattern pattern, double now, double brightnessPercent) {
    double scale = brightnessPercent / 100.0 * MAX_VALUE;
    int length = target.getLength();

    switch (pattern) {
      case OFF -> {
        for (int i = 0; i < length; i++) {
          target.setHSV(i, 0, 0, 0);
        }
      }
      case SOLID -> {
        for (int i = 0; i < length; i++) {
          target.setHSV(i, LEDConstants.SOLID_HUE, MAX_VALUE, (int) scale);
        }
      }
      case RAINBOW -> {
        int offset = (int) Math.round(now / LEDConstants.RAINBOW_FULL_CYCLE_SEC * 180.0) % 180;
        for (int i = 0; i < length; i++) {
          target.setHSV(i, (i * 180 / length + offset) % 180, MAX_VALUE, (int) scale);
        }
      }
      case CHASE -> {
        // Comet with CHASE_TAIL_PIXELS fade trailing its head, wrapping around the strip.
        double head =
            (now * LEDConstants.CHASE_SPEED_PIXELS_PER_SEC)
                % (length + LEDConstants.CHASE_TAIL_PIXELS);
        for (int i = 0; i < length; i++) {
          double distanceBehindHead = head - i;
          int value = 0;
          if (distanceBehindHead >= 0 && distanceBehindHead < LEDConstants.CHASE_TAIL_PIXELS) {
            value = (int) (scale * (1.0 - distanceBehindHead / LEDConstants.CHASE_TAIL_PIXELS));
          }
          target.setHSV(i, LEDConstants.CHASE_HUE, MAX_VALUE, value);
        }
      }
      case BREATHE -> {
        int value =
            (int)
                (scale
                    * (0.5 - 0.5 * Math.cos(2 * Math.PI * now / LEDConstants.BREATHE_PERIOD_SEC)));
        for (int i = 0; i < length; i++) {
          target.setHSV(i, LEDConstants.BREATHE_HUE, MAX_VALUE, value);
        }
      }
      default -> {
        // Theme patterns never reach this method.
      }
    }
  }

  /**
   * Publishes read-only display diagnostics for Elastic and logs the selected style and mechanism
   * state. These values never feed back into rendering. NT writes are skipped in replay.
   */
  private void publishStatus(
      LEDPattern pattern,
      IdleEffect effect,
      double brightnessPercent,
      boolean disabled,
      FlowState flowTarget) {
    String diagnostic = BallFlowTracker.diagnostic();
    String effectiveState;
    if (pattern == LEDPattern.OFF || brightnessPercent <= 0.0) {
      effectiveState = "Off";
    } else if (disabled) {
      effectiveState = "Disabled";
    } else if (diagnostic.contains("conflict")) {
      effectiveState = "Conflict";
    } else if (diagnostic.contains("local reverse")) {
      effectiveState = "LocalReverse";
    } else {
      effectiveState =
          switch (flowTarget) {
            case FORWARD -> "Forward";
            case REVERSE -> "Reverse";
            case IDLE -> "Idle";
          };
    }
    String direction =
        switch (flowTarget) {
          case FORWARD -> "Forward";
          case REVERSE -> "Reverse";
          case IDLE -> "None";
        };

    if (!Logger.hasReplaySource()) {
      SmartDashboard.putString("LED Effective State", effectiveState);
      SmartDashboard.putString("LED Flow Direction", direction);
      SmartDashboard.putString(
          "LED Driver",
          hardwareLabel(
              LEDConstants.HARDWARE,
              LEDConstants.PWM_PORT,
              LEDConstants.CANDLE_CAN_ID,
              LEDConstants.CANDLE_CAN_BUS));
    }

    Logger.recordOutput("LED/Pattern", pattern.toString());
    Logger.recordOutput("LED/Theme", pattern.theme().map(LEDTheme::toString).orElse(""));
    Logger.recordOutput("LED/IdleEffect", effect.toString());
    Logger.recordOutput("LED/FlowState", flowTarget.toString());
    Logger.recordOutput("LED/FlowDirection", direction);
    Logger.recordOutput("LED/BrightnessPercent", brightnessPercent);
    Logger.recordOutput("LED/SectionLayout", layout.describe());
    Logger.recordOutput("LED/BallFlowDiagnostic", diagnostic);
    Logger.recordOutput("LED/EffectiveState", effectiveState);
  }

  /**
   * Read-only label for the "LED Driver" dashboard widget: names the deployed driver scheme and the
   * wiring knobs it uses, so the operator can see at a glance which hardware the code expects (e.g.
   * before filling in or verifying the install parameters). Display only — switching stays a
   * code-constant change.
   */
  static String hardwareLabel(
      LEDConstants.Hardware hardware, int pwmPort, int candleCanId, String candleCanBus) {
    return switch (hardware) {
      case PWM -> "PWM (port " + pwmPort + ")";
      case CANDLE -> "CANdle (CAN " + candleCanId + ", bus " + candleCanBus + ")";
    };
  }
}
