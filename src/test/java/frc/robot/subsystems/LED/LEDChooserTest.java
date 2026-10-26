package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class LEDChooserTest {
  private static final String CHOOSER_PATH = "/SmartDashboard/LED Pattern";

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    assertNotNull(LED.mInstance, "the real LED subsystem must publish its chooser");
    SmartDashboard.updateValues();
  }

  @Test
  void ledPatternOptionsStartWithOffAndKeepMarigoldBridgeAsDefault() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    String[] expectedOptions = {"Off", "Marigold Bridge", "Festival Night", "Alebrije Glow"};

    assertAll(
        () ->
            assertArrayEquals(
                expectedOptions,
                nt.getEntry(CHOOSER_PATH + "/options").getStringArray(new String[0]),
                "Elastic must receive every pattern in the operator-facing order"),
        () ->
            assertEquals(
                "Marigold Bridge",
                nt.getEntry(CHOOSER_PATH + "/default").getString(""),
                "moving Off first must not make it the startup default"));
  }

  /**
   * Boot-restore migration (menu reduced 2026-10-26): kept names resolve to themselves, Off stays
   * Off, and every name removed from the menu — the legacy patterns and the two deleted themes —
   * resolves to the Marigold Bridge fallback so a stale settings file cannot break boot or
   * resurrect a deleted option.
   */
  @Test
  void fromDashboardNameMigratesRemovedOptionsToTheDefault() {
    for (LEDPattern pattern : LEDPattern.values()) {
      assertSame(pattern, LEDPattern.fromDashboardName(pattern.getDashboardName(), null));
    }
    String[] removed = {
      "Solid", "Rainbow", "Chase", "Breathe", "Papel Fiesta", "Candle Memory", "Not A Pattern", null
    };
    for (String saved : removed) {
      assertSame(
          LEDPattern.MARIGOLD_BRIDGE,
          LEDPattern.fromDashboardName(saved, LEDPattern.MARIGOLD_BRIDGE),
          "saved value \"" + saved + "\" must fall back to the default");
    }
    // Exact match only: a case variant of a real name is an unknown name.
    assertSame(
        LEDPattern.MARIGOLD_BRIDGE,
        LEDPattern.fromDashboardName("marigold bridge", LEDPattern.MARIGOLD_BRIDGE));
  }

  @Test
  void ledPatternComboBoxDoesNotSortOptions() throws Exception {
    JsonNode root = new ObjectMapper().readTree(Files.readString(latestLayout()));
    int matchingWidgets = 0;

    for (JsonNode tab : root.path("tabs")) {
      for (JsonNode widget : tab.path("grid_layout").path("containers")) {
        if (!widget.path("type").asText("").equals("ComboBox Chooser")
            || !widget.path("properties").path("topic").asText("").equals(CHOOSER_PATH)) {
          continue;
        }
        matchingWidgets++;
        JsonNode sortOptions = widget.path("properties").get("sort_options");
        assertTrue(
            sortOptions == null || (sortOptions.isBoolean() && !sortOptions.booleanValue()),
            "the LED Pattern ComboBox must preserve the published chooser order");
      }
    }

    assertTrue(matchingWidgets > 0, "the layout must contain the LED Pattern ComboBox");
  }

  private static Path latestLayout() throws Exception {
    try (var files = Files.list(Path.of("."))) {
      return files
          .filter(p -> p.getFileName().toString().matches("elastic-layout-\\d{4}\\.json"))
          .max(Comparator.comparing(p -> p.getFileName().toString()))
          .orElseThrow(
              () -> new IllegalStateException("no elastic-layout-MMDD.json in project root"));
    }
  }
}
