package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
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
    String[] expectedOptions = {
      "Off",
      "Marigold Bridge",
      "Solid",
      "Rainbow",
      "Chase",
      "Breathe",
      "Festival Night",
      "Papel Fiesta",
      "Alebrije Glow",
      "Candle Memory"
    };

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
