package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.BallFlowTracker.FlowKind;
import frc.robot.util.BallFlowTracker.FlowState;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

class LEDPreviewTest {
  private static final String ROOT = "/SmartDashboard/LED Preview";
  private static final String CURRENT = ROOT + "/Current Selection";
  private static final int CURRENT_SAMPLES = 64;
  private static final int CARD_SAMPLES = 32;
  private static final double NOW_SEC = 0.73;

  @ParameterizedTest(name = "{0} renders the current selection")
  @EnumSource(LEDPattern.class)
  void everyCurrentPatternMatchesItsCanonicalRender(LEDPattern pattern) {
    try (Harness harness = new Harness()) {
      harness.preview.update(pattern, IdleEffect.COLOR_STEP, NOW_SEC);

      String[] actual = harness.value(CURRENT);
      String[] expected =
          pattern
              .theme()
              .map(theme -> themeReference(theme, IdleEffect.COLOR_STEP, CURRENT_SAMPLES, NOW_SEC))
              .orElseGet(() -> legacyReference(pattern, CURRENT_SAMPLES, NOW_SEC));
      assertArrayEquals(expected, actual);
      assertEquals(CURRENT_SAMPLES, actual.length);
      assertHexFormat(actual);
    }
  }

  static Stream<Arguments> recommendedEffects() {
    return Stream.of(
        Arguments.of(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW),
        Arguments.of(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE),
        Arguments.of(LEDPattern.PAPEL_FIESTA, IdleEffect.CHASE),
        Arguments.of(LEDPattern.ALEBRIJE_GLOW, IdleEffect.SLOW_FLOW),
        Arguments.of(LEDPattern.CANDLE_MEMORY, IdleEffect.STATIC));
  }

  @ParameterizedTest(name = "{0} fixed card uses {1}")
  @MethodSource("recommendedEffects")
  void fixedCardsUseTheExactRecommendedEffects(LEDPattern pattern, IdleEffect expectedEffect) {
    LEDTheme theme = pattern.theme().orElseThrow();
    try (Harness harness = new Harness()) {
      harness.preview.update(LEDPattern.OFF, IdleEffect.STATIC, NOW_SEC);

      String[] actual = harness.value(themeTopic(theme));
      assertArrayEquals(themeReference(theme, expectedEffect, CARD_SAMPLES, NOW_SEC), actual);
      assertEquals(CARD_SAMPLES, actual.length);
      assertHexFormat(actual);
    }
  }

  static Stream<Arguments> currentTimeBehavior() {
    return Stream.of(
        Arguments.of(LEDPattern.OFF, IdleEffect.STATIC, false),
        Arguments.of(LEDPattern.SOLID, IdleEffect.STATIC, false),
        Arguments.of(LEDPattern.RAINBOW, IdleEffect.STATIC, true),
        Arguments.of(LEDPattern.CHASE, IdleEffect.STATIC, true),
        Arguments.of(LEDPattern.BREATHE, IdleEffect.STATIC, true),
        Arguments.of(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW, true),
        Arguments.of(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE, true),
        Arguments.of(LEDPattern.PAPEL_FIESTA, IdleEffect.CHASE, true),
        Arguments.of(LEDPattern.ALEBRIJE_GLOW, IdleEffect.SLOW_FLOW, true),
        Arguments.of(LEDPattern.CANDLE_MEMORY, IdleEffect.STATIC, false));
  }

  @ParameterizedTest(name = "{0} time-varying={2}")
  @MethodSource("currentTimeBehavior")
  void currentOutputDistinguishesDynamicAndStaticPatterns(
      LEDPattern pattern, IdleEffect effect, boolean expectedToChange) {
    try (Harness harness = new Harness()) {
      harness.preview.update(pattern, effect, 0.21);
      String[] initial = harness.value(CURRENT).clone();

      harness.preview.update(pattern, effect, 0.30);
      assertEquals(1, harness.writeCount(CURRENT), "a 90 ms gap must stay throttled");

      harness.preview.update(pattern, effect, 0.32);
      String[] later = harness.value(CURRENT);
      if (expectedToChange) {
        assertFalse(Arrays.equals(initial, later), pattern + " must advance with time");
        assertEquals(2, harness.writeCount(CURRENT));
      } else {
        assertArrayEquals(initial, later);
        assertEquals(1, harness.writeCount(CURRENT), "unchanged frames must not be republished");
      }
    }
  }

  @Test
  void publishesOnlyTheCurrentAndFiveNamedThemeTopics() {
    try (Harness harness = new Harness()) {
      harness.preview.update(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW, NOW_SEC);

      Set<String> expectedTopics =
          Set.of(
              CURRENT,
              themeTopic(LEDTheme.MARIGOLD_BRIDGE),
              themeTopic(LEDTheme.FESTIVAL_NIGHT),
              themeTopic(LEDTheme.PAPEL_FIESTA),
              themeTopic(LEDTheme.ALEBRIJE_GLOW),
              themeTopic(LEDTheme.CANDLE_MEMORY));
      Set<String> actualTopics =
          Arrays.stream(harness.nt.getTopics())
              .map(topic -> topic.getName())
              .collect(Collectors.toSet());
      assertEquals(expectedTopics, actualTopics);
      assertEquals(6, harness.sinkCreations);
      assertEquals(6, harness.totalWrites());
    }
  }

  @Test
  void nonRecommendedEffectOverridesOnlyTheCurrentSelection() {
    try (Harness harness = new Harness()) {
      harness.preview.update(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.STATIC, NOW_SEC);

      String[] current = harness.value(CURRENT);
      assertArrayEquals(
          themeReference(LEDTheme.MARIGOLD_BRIDGE, IdleEffect.STATIC, CURRENT_SAMPLES, NOW_SEC),
          current);
      assertArrayEquals(
          themeReference(LEDTheme.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW, CARD_SAMPLES, NOW_SEC),
          harness.value(themeTopic(LEDTheme.MARIGOLD_BRIDGE)));
      assertFalse(Arrays.equals(current, harness.value(themeTopic(LEDTheme.MARIGOLD_BRIDGE))));
    }
  }

  @Test
  void selectionChangesPublishImmediatelyWithoutTouchingFixedCards() {
    try (Harness harness = new Harness()) {
      harness.preview.update(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW, 1.0);
      String[] initialStorage = harness.lastWrite(CURRENT).storage;
      Map<String, String[]> fixedSnapshots = harness.fixedCardSnapshots();
      Map<String, Integer> fixedWriteCounts = harness.fixedCardWriteCounts();

      harness.preview.update(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE, 1.01);
      assertArrayEquals(
          themeReference(LEDTheme.FESTIVAL_NIGHT, IdleEffect.BREATHE, CURRENT_SAMPLES, 1.01),
          harness.value(CURRENT));
      assertEquals(2, harness.writeCount(CURRENT));
      harness.assertFixedCardsMatch(fixedSnapshots, fixedWriteCounts);

      harness.preview.update(LEDPattern.FESTIVAL_NIGHT, IdleEffect.STATIC, 1.02);
      assertArrayEquals(
          themeReference(LEDTheme.FESTIVAL_NIGHT, IdleEffect.STATIC, CURRENT_SAMPLES, 1.02),
          harness.value(CURRENT));
      assertEquals(3, harness.writeCount(CURRENT));
      harness.assertFixedCardsMatch(fixedSnapshots, fixedWriteCounts);
      assertSame(initialStorage, harness.lastWrite(CURRENT).storage, "String[] storage is reused");
    }
  }

  @Test
  void scheduledPublishingIsBoundedAndSuppressesUnchangedStaticCards() {
    try (Harness harness = new Harness()) {
      harness.preview.update(LEDPattern.RAINBOW, IdleEffect.STATIC, 1.0);
      Map<String, String[]> initialStorage = harness.allStorageByTopic();

      harness.preview.update(LEDPattern.RAINBOW, IdleEffect.STATIC, 1.09);
      harness.preview.update(LEDPattern.RAINBOW, IdleEffect.STATIC, 1.099);
      assertEquals(6, harness.totalWrites(), "nothing may publish before the 100 ms boundary");

      harness.preview.update(LEDPattern.RAINBOW, IdleEffect.STATIC, 1.1);
      assertEquals(2, harness.writeCount(CURRENT));
      for (LEDTheme theme : LEDTheme.values()) {
        int expectedWrites = theme == LEDTheme.CANDLE_MEMORY ? 1 : 2;
        assertEquals(expectedWrites, harness.writeCount(themeTopic(theme)), theme.toString());
      }
      assertEquals(11, harness.totalWrites(), "only the five changed frames may publish");
      for (Map.Entry<String, String[]> entry : initialStorage.entrySet()) {
        assertSame(entry.getValue(), harness.lastWrite(entry.getKey()).storage, entry.getKey());
      }

      harness.preview.update(LEDPattern.RAINBOW, IdleEffect.STATIC, 1.1001);
      assertEquals(11, harness.totalWrites());
    }
  }

  @Test
  void legacyRainbowAndChaseRenderAtFullLengthBeforeEndpointDownsampling() {
    for (LEDPattern pattern : new LEDPattern[] {LEDPattern.RAINBOW, LEDPattern.CHASE}) {
      try (Harness harness = new Harness()) {
        harness.preview.update(pattern, IdleEffect.STATIC, NOW_SEC);

        String[] expected = legacyReference(pattern, CURRENT_SAMPLES, NOW_SEC);
        assertArrayEquals(expected, harness.value(CURRENT), pattern.toString());
        assertFalse(
            Arrays.equals(expected, legacyDirectReference(pattern, CURRENT_SAMPLES, NOW_SEC)),
            "the fixture must distinguish full-length timing from a direct short render");

        AddressableLEDBuffer full = legacyFullBuffer(pattern, NOW_SEC);
        String[] actual = harness.value(CURRENT);
        assertEquals(hex(full, 0), actual[0], "the first endpoint must be preserved");
        assertEquals(
            hex(full, full.getLength() - 1),
            actual[CURRENT_SAMPLES - 1],
            "the last endpoint must be preserved");
      }
    }
  }

  @Test
  void previewIgnoresOperatorBrightnessAndMechanismAction() {
    var controls = NetworkTableInstance.getDefault();
    var brightness = controls.getEntry("/Tuning/LED Brightness (%)");
    double oldBrightness = brightness.getDouble(100.0);
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(() -> NOW_SEC);
    try (Harness harness = new Harness()) {
      brightness.setDouble(0.0);
      BallFlowTracker.report(FlowKind.INTAKE);
      assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());

      harness.preview.update(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.STATIC, NOW_SEC);
      String[] expected =
          themeReference(LEDTheme.MARIGOLD_BRIDGE, IdleEffect.STATIC, CURRENT_SAMPLES, NOW_SEC);
      assertArrayEquals(expected, harness.value(CURRENT));
      assertNotEquals("#000000", harness.value(CURRENT)[0]);

      brightness.setDouble(100.0);
      BallFlowTracker.resetForTest();
      BallFlowTracker.report(FlowKind.FULL_REVERSE);
      harness.preview.update(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.STATIC, NOW_SEC + 0.2);
      assertArrayEquals(expected, harness.value(CURRENT));
      assertEquals(1, harness.writeCount(CURRENT));
    } finally {
      brightness.setDouble(oldBrightness);
      BallFlowTracker.resetForTest();
      BallFlowTracker.setClockForTest(null);
    }
  }

  @Test
  void updatePublishesWhileDriverStationIsDisabled() {
    HAL.initialize(500, 0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    assertTrue(DriverStation.isDisabled());
    try (Harness harness = new Harness()) {
      harness.preview.update(LEDPattern.SOLID, IdleEffect.STATIC, NOW_SEC);

      assertArrayEquals(
          legacyReference(LEDPattern.SOLID, CURRENT_SAMPLES, NOW_SEC), harness.value(CURRENT));
      assertEquals(6, harness.totalWrites());
    } finally {
      DriverStationSim.setEnabled(false);
      DriverStationSim.setDsAttached(false);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void replayCreatesNoPublishersTopicsOrWrites() {
    try (Harness harness = new Harness(() -> true)) {
      for (LEDPattern pattern : LEDPattern.values()) {
        harness.preview.update(pattern, IdleEffect.COLOR_STEP, NOW_SEC);
        harness.preview.update(pattern, IdleEffect.STATIC, NOW_SEC + 0.2);
      }

      assertEquals(0, harness.sinkCreations);
      assertEquals(0, harness.totalWrites());
      assertEquals(0, harness.nt.getTopics().length);
    }
  }

  @Test
  void previewUpdatesDoNotMutateRealOnboardFlowOrControlState() throws Exception {
    HAL.initialize(500, 0);
    LED led = LED.mInstance;
    AddressableLEDBuffer real = field(led, "buffer");
    AddressableLEDBuffer onboard = field(led, "onboardBuffer");
    BallFlowLayer flow = field(led, "flowLayer");
    int[] originalReal = packed(real);
    int[] originalOnboard = packed(onboard);
    double originalPhase = flow.getPhase();
    double originalBlend = flow.getBlend();
    double originalDirection = flow.getDirectionSign();
    LEDTheme originalActiveTheme = field(led, "activeTheme");
    LEDTheme originalPreviousTheme = field(led, "previousTheme");
    double originalTransitionStart = field(led, "themeTransitionStartSec");

    var controls = NetworkTableInstance.getDefault();
    var brightness = controls.getEntry("/Tuning/LED Brightness (%)");
    var follow = controls.getEntry(LED.FOLLOW_FLOW_SETTING_KEY);
    var pattern = controls.getEntry(LED.PATTERN_SETTING_KEY + "/selected");
    var effect = controls.getEntry(LED.EFFECT_SETTING_KEY + "/selected");
    double oldBrightness = brightness.getDouble(100.0);
    boolean oldFollow = follow.getBoolean(true);
    String oldPattern = pattern.getString("");
    String oldEffect = effect.getString("");

    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(() -> NOW_SEC);
    try (Harness harness = new Harness()) {
      LED.renderLegacyInto(real, LEDPattern.RAINBOW, 0.41, 63.0);
      LED.renderLegacyInto(onboard, LEDPattern.CHASE, 0.41, 63.0);
      flow.update(0.2, FlowState.FORWARD);
      int[] sentinelReal = packed(real);
      int[] sentinelOnboard = packed(onboard);
      double sentinelPhase = flow.getPhase();
      double sentinelBlend = flow.getBlend();
      double sentinelDirection = flow.getDirectionSign();
      brightness.setDouble(23.0);
      follow.setBoolean(false);
      pattern.setString("Candle Memory");
      effect.setString("Chase");
      BallFlowTracker.report(FlowKind.INTAKE);

      harness.preview.update(LEDPattern.FESTIVAL_NIGHT, IdleEffect.COLOR_STEP, NOW_SEC);
      harness.preview.update(LEDPattern.OFF, IdleEffect.STATIC, NOW_SEC + 0.01);

      assertArrayEquals(sentinelReal, packed(real), "real strip buffer must be independent");
      assertArrayEquals(sentinelOnboard, packed(onboard), "onboard buffer must be independent");
      assertEquals(sentinelPhase, flow.getPhase());
      assertEquals(sentinelBlend, flow.getBlend());
      assertEquals(sentinelDirection, flow.getDirectionSign());
      assertSame(originalActiveTheme, field(led, "activeTheme"));
      assertSame(originalPreviousTheme, field(led, "previousTheme"));
      assertEquals(originalTransitionStart, (double) field(led, "themeTransitionStartSec"));
      assertEquals(23.0, brightness.getDouble(0.0));
      assertFalse(follow.getBoolean(true));
      assertEquals("Candle Memory", pattern.getString(""));
      assertEquals("Chase", effect.getString(""));
      assertEquals(FlowState.FORWARD, BallFlowTracker.effectiveState());
      assertEquals("", BallFlowTracker.diagnostic());
    } finally {
      restoreBuffer(real, originalReal);
      restoreBuffer(onboard, originalOnboard);
      setField(flow, "phase", originalPhase);
      setField(flow, "blend", originalBlend);
      setField(flow, "directionSign", originalDirection);
      setField(led, "activeTheme", originalActiveTheme);
      setField(led, "previousTheme", originalPreviousTheme);
      setField(led, "themeTransitionStartSec", originalTransitionStart);
      brightness.setDouble(oldBrightness);
      follow.setBoolean(oldFollow);
      pattern.setString(oldPattern);
      effect.setString(oldEffect);
      BallFlowTracker.resetForTest();
      BallFlowTracker.setClockForTest(null);
    }
  }

  private static String themeTopic(LEDTheme theme) {
    return ROOT + "/" + theme.getDashboardName();
  }

  private static String[] themeReference(
      LEDTheme theme, IdleEffect effect, int samples, double nowSec) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(samples);
    LEDSection[] sections = {new LEDSection(0, samples, false, LEDSection.Role.BALL_PATH)};
    LED.renderThemeInto(
        buffer, sections, theme::colorAt, effect, new BallFlowLayer(), nowSec, 1.0, false);
    return hex(buffer);
  }

  private static String[] legacyReference(LEDPattern pattern, int samples, double nowSec) {
    AddressableLEDBuffer full = legacyFullBuffer(pattern, nowSec);
    String[] result = new String[samples];
    for (int i = 0; i < samples; i++) {
      int source =
          samples == 1 ? 0 : (int) Math.round((double) i * (full.getLength() - 1) / (samples - 1));
      result[i] = hex(full, source);
    }
    return result;
  }

  private static String[] legacyDirectReference(LEDPattern pattern, int samples, double nowSec) {
    AddressableLEDBuffer direct = new AddressableLEDBuffer(samples);
    LED.renderLegacyInto(direct, pattern, nowSec, 100.0);
    return hex(direct);
  }

  private static AddressableLEDBuffer legacyFullBuffer(LEDPattern pattern, double nowSec) {
    AddressableLEDBuffer full = new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);
    LED.renderLegacyInto(full, pattern, nowSec, 100.0);
    return full;
  }

  private static String[] hex(AddressableLEDBuffer buffer) {
    String[] result = new String[buffer.getLength()];
    for (int i = 0; i < result.length; i++) {
      result[i] = hex(buffer, i);
    }
    return result;
  }

  private static String hex(AddressableLEDBuffer buffer, int index) {
    return String.format(
        "#%02X%02X%02X", buffer.getRed(index), buffer.getGreen(index), buffer.getBlue(index));
  }

  private static void assertHexFormat(String[] values) {
    for (String value : values) {
      assertTrue(value.matches("#[0-9A-F]{6}"), "invalid color: " + value);
    }
  }

  private static int[] packed(AddressableLEDBuffer buffer) {
    int[] result = new int[buffer.getLength()];
    for (int i = 0; i < result.length; i++) {
      result[i] = (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
    }
    return result;
  }

  private static void restoreBuffer(AddressableLEDBuffer buffer, int[] colors) {
    for (int i = 0; i < colors.length; i++) {
      buffer.setRGB(i, (colors[i] >> 16) & 0xFF, (colors[i] >> 8) & 0xFF, colors[i] & 0xFF);
    }
  }

  @SuppressWarnings("unchecked")
  private static <T> T field(Object target, String name) throws ReflectiveOperationException {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return (T) field.get(target);
  }

  private static void setField(Object target, String name, Object value)
      throws ReflectiveOperationException {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    field.set(target, value);
  }

  private record Write(String[] storage) {}

  private static final class Harness implements AutoCloseable {
    final NetworkTableInstance nt = NetworkTableInstance.create();
    final Map<String, List<Write>> writes = new LinkedHashMap<>();
    final List<StringArrayPublisher> publishers = new ArrayList<>();
    final LEDPreview preview;
    int sinkCreations;

    Harness() {
      this(() -> false);
    }

    Harness(BooleanSupplier replaySupplier) {
      preview = new LEDPreview(nt, replaySupplier, this::createSink);
    }

    private LEDPreview.PreviewSink createSink(String topic) {
      sinkCreations++;
      StringArrayPublisher publisher = nt.getStringArrayTopic(topic).publish();
      publishers.add(publisher);
      return values -> {
        writes.computeIfAbsent(topic, unused -> new ArrayList<>()).add(new Write(values));
        publisher.set(values);
      };
    }

    String[] value(String topic) {
      return nt.getEntry(topic).getStringArray(new String[0]);
    }

    int writeCount(String topic) {
      return writes.getOrDefault(topic, List.of()).size();
    }

    int totalWrites() {
      return writes.values().stream().mapToInt(List::size).sum();
    }

    Write lastWrite(String topic) {
      List<Write> topicWrites = writes.get(topic);
      assertTrue(topicWrites != null && !topicWrites.isEmpty(), "no writes for " + topic);
      return topicWrites.get(topicWrites.size() - 1);
    }

    Map<String, String[]> fixedCardSnapshots() {
      Map<String, String[]> result = new LinkedHashMap<>();
      for (LEDTheme theme : LEDTheme.values()) {
        result.put(themeTopic(theme), value(themeTopic(theme)).clone());
      }
      return result;
    }

    Map<String, Integer> fixedCardWriteCounts() {
      Map<String, Integer> result = new LinkedHashMap<>();
      for (LEDTheme theme : LEDTheme.values()) {
        result.put(themeTopic(theme), writeCount(themeTopic(theme)));
      }
      return result;
    }

    Map<String, String[]> allStorageByTopic() {
      Map<String, String[]> result = new LinkedHashMap<>();
      for (String topic : writes.keySet()) {
        result.put(topic, lastWrite(topic).storage);
      }
      return result;
    }

    void assertFixedCardsMatch(
        Map<String, String[]> expectedValues, Map<String, Integer> expectedWriteCounts) {
      for (LEDTheme theme : LEDTheme.values()) {
        String topic = themeTopic(theme);
        assertArrayEquals(expectedValues.get(topic), value(topic), topic);
        assertEquals(expectedWriteCounts.get(topic), writeCount(topic), topic);
      }
    }

    @Override
    public void close() {
      for (StringArrayPublisher publisher : publishers) {
        publisher.close();
      }
      nt.close();
    }
  }
}
