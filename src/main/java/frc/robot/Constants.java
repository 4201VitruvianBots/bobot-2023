// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.CONSTANTS.kCANCoderSensorUnitsPerRotation;
import static frc.robot.Constants.CONSTANTS.kFalconSensorUnitsPerRotation;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static String robotName = "";

  // Add any constants that do not change between robots here, as well as all
  // enums

  public static class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;
  }

  public static final class CAN {
    public static final int pigeon = 9;

    public static final int frontLeftCanCoder = 10;
    public static final int frontRightCanCoder = 11;
    public static final int backLeftCanCoder = 12;
    public static final int backRightCanCoder = 13;

    public static final int frontLeftDriveMotor = 20;
    public static final int frontLeftTurnMotor = 21;
    public static final int frontRightDriveMotor = 22;
    public static final int frontRightTurnMotor = 23;
    public static final int backLeftDriveMotor = 24;
    public static final int backLeftTurnMotor = 25;
    public static final int backRightDriveMotor = 26;
    public static final int backRightTurnMotor = 27;

    public static final int wristMotor = 30;
    public static final int intakeMotor = 31;
    public static final int flywheelMotor = 32;
  }

  public static final class CONSTANTS {
    public static final int kFalconSensorUnitsPerRotation = 2048;
    public static final int kCANCoderSensorUnitsPerRotation = 4096;
  }

  public static final class INTAKE {
    public static final double innerIntakeWidth = Units.inchesToMeters(15.5);
    public static final int leftConeSensorId = 1;
    public static final int rightConeSensorId = 2;
    public static final int cubeSensorId = 3;
    public static final double length = Units.inchesToMeters(12);

    public static final double gearRatio = 48.0 / 16.0;
    public static final DCMotor gearBox = DCMotor.getFalcon500(1);
    public static final double kMotorDistancePerPulse =
        360.0 / (kFalconSensorUnitsPerRotation * gearRatio);

    public static double kF = 0;
    public static double kP = 0.2;
  }

  public static final class SWERVE_DRIVE {
    public static final double kTrackWidth = Units.inchesToMeters(24);
    public static final double kWheelBase = Units.inchesToMeters(24);

    public static final Map<SWERVE_MODULE_POSITION, Translation2d> kModuleTranslations =
        Map.of(
            SWERVE_MODULE_POSITION.FRONT_LEFT,
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            SWERVE_MODULE_POSITION.FRONT_RIGHT,
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            SWERVE_MODULE_POSITION.BACK_LEFT,
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            SWERVE_MODULE_POSITION.BACK_RIGHT,
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

    public static double frontLeftCANCoderOffset = 125.068;
    public static double frontRightCANCoderOffset = 62.051;
    public static double backLeftCANCoderOffset = 190.635;
    public static double backRightCANCoderOffset = 31.904;

    public static double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kLimitedSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 5;
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;
    public static final double kLimitedRotationRadiansPerSecond = kMaxRotationRadiansPerSecond / 5;

    public static final double kP_X = 0.6;
    public static final double kI_X = 0;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.6;
    public static final double kI_Y = 0;
    public static final double kD_Y = 0;

    public static double kP_Theta = 4.0;
    public static double kI_Theta = 0;
    public static double kD_Theta = 0.01;

    public enum SWERVE_MODULE_POSITION {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }

  public static class SWERVE_MODULE {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kDriveMotorDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kFalconSensorUnitsPerRotation * kDriveMotorGearRatio);
    public static final double kTurningMotorDistancePerPulse =
        360.0 / (kFalconSensorUnitsPerRotation * kTurningMotorGearRatio);
    public static final double kTurnEncoderDistancePerPulse =
        360.0 / kCANCoderSensorUnitsPerRotation;

    public static final double ksDriveVoltSecondsPerMeter = 0.605 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 1.72 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.193 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }

  public static final class VISION {
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
      INTAKE("10.42.1.11"),
      LEFT_LOCALIZER("10.42.1.12"),
      RIGHT_LOCALIZER("10.42.1.13"),
      FUSED_LOCALIZER("10.42.1.12");

      private final String ip;

      CAMERA_SERVER(final String ip) {
        this.ip = ip;
      }

      @Override
      public String toString() {
        return ip;
      }
    }

    public static final Transform3d[] LOCALIZER_CAMERA_POSITION = {
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

  public static class AUTO {
    public static double kAutoBalanceTimeout = 2.0;
    public static final double kAutoBalanceAngleThresholdDegrees = 2.0;
  }

  public enum SETPOINT {
    // Units are in Radians

    STOWED(Units.degreesToRadians(98.0)),
    INTAKING_LOW_CUBE(Units.degreesToRadians(-13.5)),
    SCORE_LOW_REVERSE(Units.degreesToRadians(-14.0)),
    SCORE_HIGH_CUBE(Units.degreesToRadians(147.0)),
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

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }
}
