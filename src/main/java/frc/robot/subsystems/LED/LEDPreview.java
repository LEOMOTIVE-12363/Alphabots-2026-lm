package frc.robot.subsystems.LED;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Independent, reference-brightness LED previews for Elastic Multi Color View widgets. */
final class LEDPreview {
  static final int CURRENT_SELECTION_SAMPLES = 64;
  static final int THEME_CARD_SAMPLES = 32;
  static final double PUBLISH_PERIOD_SEC = 0.1;

  private static final String ROOT = "/SmartDashboard/LED Preview";
  private static final String CURRENT_TOPIC = ROOT + "/Current Selection";
  private static final LEDSection[] CURRENT_SECTIONS = {
    new LEDSection(0, CURRENT_SELECTION_SAMPLES, false, LEDSection.Role.BALL_PATH)
  };
  private static final LEDSection[] THEME_CARD_SECTIONS = {
    new LEDSection(0, THEME_CARD_SAMPLES, false, LEDSection.Role.BALL_PATH)
  };

  private final BooleanSupplier replaySupplier;
  private final PreviewSinkFactory sinkFactory;
  private final PreviewSink[] sinks = new PreviewSink[1 + LEDTheme.values().length];
  private final AddressableLEDBuffer currentBuffer =
      new AddressableLEDBuffer(CURRENT_SELECTION_SAMPLES);
  private final AddressableLEDBuffer[] themeBuffers =
      new AddressableLEDBuffer[LEDTheme.values().length];
  private final AddressableLEDBuffer legacyBuffer =
      new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);
  private final BallFlowLayer flowPlaceholder = new BallFlowLayer();
  private final String[] currentOutput = new String[CURRENT_SELECTION_SAMPLES];
  private final String[][] themeOutputs = new String[LEDTheme.values().length][];
  private final int[] currentPacked = new int[CURRENT_SELECTION_SAMPLES];
  private final int[][] themePacked = new int[LEDTheme.values().length][];
  private final boolean[] published = new boolean[1 + LEDTheme.values().length];

  private boolean sinksReady;
  private double lastScheduledPublishSec = Double.NaN;
  private LEDPattern lastPattern;
  private IdleEffect lastEffect;

  LEDPreview() {
    this(NetworkTableInstance.getDefault(), Logger::hasReplaySource);
  }

  LEDPreview(NetworkTableInstance nt, BooleanSupplier replaySupplier) {
    this(nt, replaySupplier, topic -> createNtSink(nt, topic));
  }

  LEDPreview(
      NetworkTableInstance nt, BooleanSupplier replaySupplier, PreviewSinkFactory sinkFactory) {
    this.replaySupplier = replaySupplier;
    this.sinkFactory = sinkFactory;
    for (int i = 0; i < LEDTheme.values().length; i++) {
      themeBuffers[i] = new AddressableLEDBuffer(THEME_CARD_SAMPLES);
      themeOutputs[i] = new String[THEME_CARD_SAMPLES];
      themePacked[i] = new int[THEME_CARD_SAMPLES];
    }
  }

  void update(LEDPattern pattern, IdleEffect effect, double nowSec) {
    if (replaySupplier.getAsBoolean()) {
      return;
    }

    boolean selectionChanged = pattern != lastPattern || effect != lastEffect;
    boolean scheduled =
        !Double.isFinite(lastScheduledPublishSec)
            || nowSec < lastScheduledPublishSec
            || nowSec - lastScheduledPublishSec >= PUBLISH_PERIOD_SEC;
    if (!scheduled && !selectionChanged) {
      return;
    }
    ensureSinks();

    if (scheduled) {
      lastScheduledPublishSec = nowSec;
      renderCurrent(pattern, effect, nowSec);
      publishBuffer(0, currentBuffer, currentOutput, currentPacked, selectionChanged);
      for (LEDTheme theme : LEDTheme.values()) {
        int index = theme.ordinal();
        LED.renderThemeInto(
            themeBuffers[index],
            THEME_CARD_SECTIONS,
            theme::colorAt,
            theme.getRecommendedEffect(),
            flowPlaceholder,
            nowSec,
            1.0,
            false);
        publishBuffer(
            index + 1, themeBuffers[index], themeOutputs[index], themePacked[index], false);
      }
    } else {
      renderCurrent(pattern, effect, nowSec);
      publishBuffer(0, currentBuffer, currentOutput, currentPacked, true);
    }

    lastPattern = pattern;
    lastEffect = effect;
  }

  private void renderCurrent(LEDPattern pattern, IdleEffect effect, double nowSec) {
    if (pattern.theme().isPresent()) {
      LED.renderThemeInto(
          currentBuffer,
          CURRENT_SECTIONS,
          pattern.theme().get()::colorAt,
          effect,
          flowPlaceholder,
          nowSec,
          1.0,
          false);
      return;
    }

    LED.renderLegacyInto(legacyBuffer, pattern, nowSec, 100.0);
    for (int i = 0; i < CURRENT_SELECTION_SAMPLES; i++) {
      int source =
          CURRENT_SELECTION_SAMPLES == 1
              ? 0
              : (int)
                  Math.round(
                      (double) i
                          * (legacyBuffer.getLength() - 1)
                          / (CURRENT_SELECTION_SAMPLES - 1));
      currentBuffer.setRGB(
          i,
          legacyBuffer.getRed(source),
          legacyBuffer.getGreen(source),
          legacyBuffer.getBlue(source));
    }
  }

  private void ensureSinks() {
    if (sinksReady) {
      return;
    }
    sinks[0] = sinkFactory.create(CURRENT_TOPIC);
    for (LEDTheme theme : LEDTheme.values()) {
      sinks[theme.ordinal() + 1] = sinkFactory.create(ROOT + "/" + theme.getDashboardName());
    }
    sinksReady = true;
  }

  private void publishBuffer(
      int sinkIndex,
      AddressableLEDBuffer buffer,
      String[] output,
      int[] previousPacked,
      boolean forcePublish) {
    boolean changed = !published[sinkIndex];
    for (int i = 0; i < buffer.getLength(); i++) {
      int packed = (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
      if (!published[sinkIndex] || previousPacked[i] != packed) {
        output[i] = String.format("#%06X", packed);
        previousPacked[i] = packed;
        changed = true;
      }
    }
    if (changed || forcePublish) {
      sinks[sinkIndex].set(output);
      published[sinkIndex] = true;
    }
  }

  private static PreviewSink createNtSink(NetworkTableInstance nt, String topic) {
    StringArrayPublisher publisher = nt.getStringArrayTopic(topic).publish();
    return publisher::set;
  }

  @FunctionalInterface
  interface PreviewSink {
    void set(String[] values);
  }

  @FunctionalInterface
  interface PreviewSinkFactory {
    PreviewSink create(String topic);
  }
}
