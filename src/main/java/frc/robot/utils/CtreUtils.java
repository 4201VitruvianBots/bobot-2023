package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class CtreUtils {
  public static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kP = 0.6; // 0.8;
    motorConfig.Slot0.kI = 0.0001;
    //    motorConfig.Slot0.integralZone = 121.904762;
    motorConfig.Slot0.kD = 12; // 0.0;
    //    motorConfig.Slot0.allowableClosedloopError = 0.0;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //    motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    return motorConfig;
  }

  public static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kP = 0.1;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25; // TODO adjust this later
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; // TODO Adjust this later

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    //    motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    return motorConfig;
  }

  public static CANcoderConfiguration generateCanCoderConfig() {
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    //    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    sensorConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Unsigned_0To1; // TODO Adjust code for this
    //    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    return sensorConfig;
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
