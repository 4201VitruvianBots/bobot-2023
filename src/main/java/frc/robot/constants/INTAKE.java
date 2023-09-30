package frc.robot.constants;

import static frc.robot.constants.BASE.CONSTANTS.kFalconSensorUnitsPerRotation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class INTAKE {
  public static final double innerIntakeWidth = Units.inchesToMeters(15.5);
  public static final int leftConeSensorId = 1;
  public static final int rightConeSensorId = 2;
  public static final int cubeSensorId = 3;
  public static final double length = Units.inchesToMeters(12);

  public static final double gearRatio = 48.0 / 16.0;
  public static final DCMotor gearBox = DCMotor.getFalcon500(1);
  public static final double kMotorDistancePerPulse =
      360.0 / (kFalconSensorUnitsPerRotation * gearRatio);

  public static double kV = 0;
  public static double kP = 0.2;
}
