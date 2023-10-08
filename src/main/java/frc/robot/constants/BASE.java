package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class BASE {
  public static String robotName = "";

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public static final class CONSTANTS {
    public static final int kFalconSensorUnitsPerRotation = 2048;
    public static final int kNeoSensorUnitsPerRotation = 42;
    public static final int kCANCoderSensorUnitsPerRotation = 4096;

    public static final double gearRatio = 60 / 1;
    public static final double kFalconEncoderUnitsToDegrees =
        360.0 / (kFalconSensorUnitsPerRotation * gearRatio);

    public static final double kNeoEncoderUnitsToDegrees =
        360 / gearRatio; // Gear ratio seems to be already accounted for, thanks Jadon!
    public static final double kPercentOutputMultiplier = 0.2;
  }

  public enum SETPOINT {
    // Units are in Radians

    STOWED(Units.degreesToRadians(0.0)), // Temporary value
    INTAKING_LOW_CUBE(Units.degreesToRadians(164.0)), // Temporary value
    SCORE_LOW_REVERSE(Units.degreesToRadians(145.0)), // Temporary value
    SCORE_MID_CUBE(Units.degreesToRadians(28.0)), // Temporary value
    SCORE_HIGH_CUBE(Units.degreesToRadians(28.0)), // Temporary value
    INTAKING_EXTENDED_CUBE(SCORE_HIGH_CUBE.get());

    public double getWristSetpointRadians() {
      return value;
    }

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
