package frc.robot.subsystems.LED;

import edu.wpi.first.math.MathUtil;

/**
 * One physically contiguous run of the LED strip and its role in the logical layout. Sections are
 * described in absolute buffer pixels; {@code reversed} flips the logical direction so two mirrored
 * ball-path strips still show the same physical gradient direction: u is always 0 at the intake
 * entrance and 1 at the shooter top, wherever those are physically wired.
 */
public record LEDSection(int startPixel, int lengthPixels, boolean reversed, Role role) {

  /** What this run of pixels displays. */
  public enum Role {
    /** Part of the intake→shooter path. */
    BALL_PATH,
    /** Decoration-only run; it uses the same selected dashboard style. */
    DECORATION
  }

  /**
   * Section-local logical coordinate in [0, 1] for an absolute buffer index (honors {@code
   * reversed}): the first pixel of a forward section is u=0 and the last is u=1. Indexes outside
   * the section clamp to the nearest end.
   */
  public double uForPixel(int pixelIndex) {
    double u = lengthPixels <= 1 ? 0.0 : (double) (pixelIndex - startPixel) / (lengthPixels - 1);
    if (reversed) {
      u = 1.0 - u;
    }
    return MathUtil.clamp(u, 0.0, 1.0);
  }
}
