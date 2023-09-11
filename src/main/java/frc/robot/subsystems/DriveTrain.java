// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants.DriveTrainNeutralMode;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  double m_leftOutput, m_rightOutput;

  private final Pigeon2 pigeon = new Pigeon2(Constants.DriveTrainConstants.pigeonID, "rio");
  private double lastYaw = 0;
  private double yawPerSecond = 0;
  private DriveTrainNeutralMode neutralMode = DriveTrainNeutralMode.COAST;
  private double leftOutput = 0, rightOutput = 0, magnitude = 1.0;
  private TalonSRX[] driveMotors = {
    new TalonSRX(21),
    new TalonSRX(23),
    new TalonSRX(22),
    new TalonSRX(24),
};

  public DriveTrain() {
    for (TalonSRX motor : driveMotors) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Coast);
  }

  driveMotors[0].setInverted(true);
  driveMotors[1].setInverted(true);
  driveMotors[2].setInverted(false);
  driveMotors[3].setInverted(false);

  driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
  driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());

  driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void resetEncoders() {
    driveMotors[0].setSelectedSensorPosition(0);
    driveMotors[2].setSelectedSensorPosition(0);
  }

  public ControlMode getMotorControlMode() {
  return driveMotors[0].getControlMode();
  }

  public void setMotorPercentOutput(double leftOutput, double rightOutput) {
    driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
    driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
  }

  public void setMotorArcadeDrive(double throttle) {
  leftOutput = throttle; 
  rightOutput = throttle; 

  // Normalization
  magnitude = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
  if (magnitude > 1.0) {
    leftOutput /= magnitude;
    rightOutput /= magnitude;
  }

  setMotorPercentOutput(leftOutput, rightOutput);
  // setMotorTankDrive(leftOutput, rightOutput);
  }
  
  public void setDriveTrainNeutralMode(DriveTrainNeutralMode mode) {
    neutralMode = mode;
    switch (mode) {
      case COAST:
        for (TalonSRX motor : driveMotors) motor.setNeutralMode(NeutralMode.Coast);
        break;
      case BRAKE:
        for (TalonSRX motor : driveMotors) motor.setNeutralMode(NeutralMode.Brake);
        break;
      case HALF_BRAKE:
      default:
      driveMotors[0].setNeutralMode(NeutralMode.Brake);
      driveMotors[1].setNeutralMode(NeutralMode.Coast);
      driveMotors[2].setNeutralMode(NeutralMode.Brake);
      driveMotors[3].setNeutralMode(NeutralMode.Coast);        
      break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
