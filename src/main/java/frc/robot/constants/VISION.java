package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VISION {
  public enum CAMERA_TYPE {
    OAK,
    LIMELIGHT,
    PHOTONVISION
  }

  public enum PIPELINE {
    DEFAULT(0),
    CUBE(1),
    CONE(2);

    private final int pipeline;

    PIPELINE(final int pipeline) {
      this.pipeline = pipeline;
    }

    public int get() {
      return pipeline;
    }
  }

  public enum CAMERA_SERVER {
    LIMELIGHT("10.42.1.11");

    private final String ip;

    CAMERA_SERVER(final String ip) {
      this.ip = ip;
    }

    @Override
    public String toString() {
      return ip;
    }
  }

  public static final Transform3d[] CAMERA_POSITION = {
    // Robot Center to Left Camera
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-(3 + (3.0 / 8.0))),
            Units.inchesToMeters(12),
            Units.inchesToMeters(20)),
        new Rotation3d()),
    // Robot Center to Right Camera
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-(3 + (3.0 / 8.0))),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(20)),
        new Rotation3d()),
  };
}
