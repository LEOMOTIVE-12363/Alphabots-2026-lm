package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class LEDPreviewLayoutTest {
  private static final String PREVIEW_ROOT = "/SmartDashboard/LED Preview";
  private static final String CURRENT_TOPIC = PREVIEW_ROOT + "/Current Selection";
  private static final String PATTERN_TOPIC = "/SmartDashboard/LED Pattern";
  private static final String EFFECT_TOPIC = "/SmartDashboard/LED Idle Effect";
  private static final String BRIGHTNESS_DISCLAIMER = "Preview colors — not real-strip brightness";
  private static final List<ThemeCard> THEME_CARDS =
      List.of(
          new ThemeCard(LEDTheme.MARIGOLD_BRIDGE, 0.0),
          new ThemeCard(LEDTheme.FESTIVAL_NIGHT, 256.0),
          new ThemeCard(LEDTheme.PAPEL_FIESTA, 512.0),
          new ThemeCard(LEDTheme.ALEBRIJE_GLOW, 768.0),
          new ThemeCard(LEDTheme.CANDLE_MEMORY, 1024.0));

  private static JsonNode root;

  @BeforeAll
  static void loadLayout() throws Exception {
    try (var files = Files.list(Path.of("."))) {
      List<Path> layouts =
          files
              .filter(p -> p.getFileName().toString().matches("elastic-layout-\\d{4}\\.json"))
              .toList();
      assertEquals(1, layouts.size(), "exactly one dated Elastic layout must exist");
      root = new ObjectMapper().readTree(Files.readString(layouts.get(0)));
    }
  }

  @Test
  void existingTabsAndWidgetCountsArePreserved() {
    List<String> tabNames = new ArrayList<>();
    for (JsonNode tab : root.path("tabs")) {
      tabNames.add(tab.path("name").asText());
    }

    assertEquals(List.of("Teleoperated", "DriverDashboard", "demo", "LED Preview"), tabNames);
    assertAll(
        () -> assertEquals(0, containers(tab("Teleoperated")).size()),
        () -> assertEquals(18, containers(tab("DriverDashboard")).size()),
        () -> assertEquals(22, containers(tab("demo")).size()));
  }

  @Test
  void previewTabHasExactlyTwoExistingChoosersAndSixUniqueColorViews() {
    List<JsonNode> widgets = containers(tab("LED Preview"));
    List<JsonNode> choosers = widgetsOfType(widgets, "ComboBox Chooser");
    List<JsonNode> colorViews = widgetsOfType(widgets, "Multi Color View");

    assertAll(
        () -> assertEquals(8, widgets.size()),
        () -> assertEquals(2, choosers.size()),
        () -> assertEquals(6, colorViews.size()),
        () ->
            assertEquals(
                8,
                widgets.stream().map(w -> w.path("title").asText()).distinct().count(),
                "every preview widget must have a unique title"));
    assertEquals(Set.of(PATTERN_TOPIC, EFFECT_TOPIC), topics(choosers));
    for (JsonNode chooser : choosers) {
      assertNativeChooserContract(chooser);
    }

    Set<String> actualTopics = topics(colorViews);
    assertEquals(6, actualTopics.size(), "preview topics must be unique");
    assertEquals(expectedPreviewTopics(), actualTopics);
  }

  @Test
  void allFourLedChoosersUseNativeElasticWidgetContract() {
    List<JsonNode> choosers = new ArrayList<>();
    for (String tabName : List.of("demo", "LED Preview")) {
      for (JsonNode widget : containers(tab(tabName))) {
        String topic = widget.path("properties").path("topic").asText();
        if (topic.equals(PATTERN_TOPIC) || topic.equals(EFFECT_TOPIC)) {
          choosers.add(widget);
        }
      }
    }

    assertEquals(4, choosers.size());
    for (JsonNode chooser : choosers) {
      assertNativeChooserContract(chooser);
    }
  }

  private static void assertNativeChooserContract(JsonNode chooser) {
    JsonNode properties = chooser.path("properties");
    String title = chooser.path("title").asText();
    assertAll(
        () -> assertEquals("ComboBox Chooser", chooser.path("type").asText(), title),
        () -> assertTrue(properties.path("sort_options").isBoolean(), title + " sort_options"),
        () -> assertEquals(false, properties.path("sort_options").asBoolean(), title),
        () -> assertTrue(properties.get("data_type") == null, title + " legacy data_type"));
  }

  @Test
  void colorViewsUseStringArraysAndThePublisherPeriod() {
    for (JsonNode widget : widgetsOfType(containers(tab("LED Preview")), "Multi Color View")) {
      String topic = widget.path("properties").path("topic").asText();
      assertAll(
          () -> assertEquals("string[]", widget.path("properties").path("data_type").asText()),
          () -> assertEquals(0.1, widget.path("properties").path("period").asDouble()),
          () ->
              assertEquals(
                  LEDPreview.PUBLISH_PERIOD_SEC,
                  widget.path("properties").path("period").asDouble(),
                  topic + " must match the renderer publish period"));
    }
  }

  @Test
  void publisherSampleCountsMatchThePreviewContract() {
    assertAll(
        () -> assertEquals(64, LEDPreview.CURRENT_SELECTION_SAMPLES),
        () -> assertEquals(32, LEDPreview.THEME_CARD_SAMPLES),
        () -> assertEquals(0.1, LEDPreview.PUBLISH_PERIOD_SEC));
  }

  @Test
  void previewWidgetsHaveExactAlignedGeometryWithoutOverlaps() {
    JsonNode preview = tab("LED Preview");
    assertBounds(widgetByTopic(preview, PATTERN_TOPIC), 0.0, 0.0, 256.0, 128.0);
    assertBounds(widgetByTopic(preview, EFFECT_TOPIC), 256.0, 0.0, 256.0, 128.0);
    assertBounds(widgetByTopic(preview, CURRENT_TOPIC), 0.0, 128.0, 1280.0, 256.0);
    for (ThemeCard card : THEME_CARDS) {
      assertBounds(widgetByTopic(preview, topic(card)), card.x(), 384.0, 256.0, 128.0);
    }

    List<JsonNode> widgets = containers(preview);
    for (JsonNode widget : widgets) {
      String title = widget.path("title").asText();
      for (String field : List.of("x", "y", "width", "height")) {
        double value = widget.path(field).asDouble();
        assertTrue(value % 128.0 == 0.0, title + " has unaligned " + field + "=" + value);
      }
      assertTrue(widget.path("width").asDouble() > 0.0, title + " has non-positive width");
      assertTrue(widget.path("height").asDouble() > 0.0, title + " has non-positive height");
    }

    for (int i = 0; i < widgets.size(); i++) {
      for (int j = i + 1; j < widgets.size(); j++) {
        JsonNode a = widgets.get(i);
        JsonNode b = widgets.get(j);
        boolean separated =
            a.path("x").asDouble() + a.path("width").asDouble() <= b.path("x").asDouble()
                || b.path("x").asDouble() + b.path("width").asDouble() <= a.path("x").asDouble()
                || a.path("y").asDouble() + a.path("height").asDouble() <= b.path("y").asDouble()
                || b.path("y").asDouble() + b.path("height").asDouble() <= a.path("y").asDouble();
        assertTrue(
            separated,
            "overlapping widgets: "
                + a.path("title").asText()
                + " and "
                + b.path("title").asText());
      }
    }
  }

  @Test
  void currentSelectionTitleStatesPurposeAndBrightnessDisclaimer() {
    String title = widgetByTopic(tab("LED Preview"), CURRENT_TOPIC).path("title").asText();

    assertAll(
        () ->
            assertTrue(
                title.contains("Current Selection"),
                "current card must identify its purpose; actual title: " + title),
        () ->
            assertTrue(
                title.contains(BRIGHTNESS_DISCLAIMER),
                "current card must retain its brightness disclaimer; actual title: " + title));
  }

  @Test
  void fixedCardsLabelTheirSourceRecommendedEffects() {
    JsonNode preview = tab("LED Preview");
    for (ThemeCard card : THEME_CARDS) {
      assertEquals(
          card.theme().getDashboardName()
              + " — "
              + card.theme().getRecommendedEffect().getDashboardName(),
          widgetByTopic(preview, topic(card)).path("title").asText());
    }
  }

  private static JsonNode tab(String name) {
    for (JsonNode tab : root.path("tabs")) {
      if (tab.path("name").asText().equals(name)) {
        return tab;
      }
    }
    throw new AssertionError("missing tab: " + name);
  }

  private static List<JsonNode> containers(JsonNode tab) {
    List<JsonNode> containers = new ArrayList<>();
    for (JsonNode widget : tab.path("grid_layout").path("containers")) {
      containers.add(widget);
    }
    return containers;
  }

  private static List<JsonNode> widgetsOfType(List<JsonNode> widgets, String type) {
    return widgets.stream().filter(w -> w.path("type").asText().equals(type)).toList();
  }

  private static JsonNode widgetByTopic(JsonNode tab, String topic) {
    JsonNode match = null;
    for (JsonNode widget : containers(tab)) {
      if (!widget.path("properties").path("topic").asText().equals(topic)) {
        continue;
      }
      if (match != null) {
        throw new AssertionError("duplicate widget for topic: " + topic);
      }
      match = widget;
    }
    if (match == null) {
      throw new AssertionError("missing widget for topic: " + topic);
    }
    return match;
  }

  private static Set<String> topics(List<JsonNode> widgets) {
    return widgets.stream()
        .map(w -> w.path("properties").path("topic").asText())
        .collect(Collectors.toSet());
  }

  private static Set<String> expectedPreviewTopics() {
    Set<String> expected = new LinkedHashSet<>();
    expected.add(CURRENT_TOPIC);
    for (ThemeCard card : THEME_CARDS) {
      expected.add(topic(card));
    }
    return expected;
  }

  private static String topic(ThemeCard card) {
    return PREVIEW_ROOT + "/" + card.theme().getDashboardName();
  }

  private static void assertBounds(
      JsonNode widget, double x, double y, double width, double height) {
    String title = widget.path("title").asText();
    assertAll(
        () -> assertEquals(x, widget.path("x").asDouble(), title + " x"),
        () -> assertEquals(y, widget.path("y").asDouble(), title + " y"),
        () -> assertEquals(width, widget.path("width").asDouble(), title + " width"),
        () -> assertEquals(height, widget.path("height").asDouble(), title + " height"));
  }

  private record ThemeCard(LEDTheme theme, double x) {}
}
