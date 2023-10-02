// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;

public class IntakeShooter extends SubsystemBase {

  private final CANSparkMax kickerMotor = new CANSparkMax(CAN.kickerMotor, MotorType.kBrushless);
  private final CANSparkMax flywheelMotor = new CANSparkMax(CAN.flywheelMotor, MotorType.kBrushless);

  private final CANSparkMax[] motors = {kickerMotor, flywheelMotor};

  private DoublePublisher kickerOutputPub, flywheelOutputPub;

  public IntakeShooter() {
    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
    }

    flywheelMotor.getPIDController().setP(INTAKE.kP);
    
    // Shuffleboard setup
    NetworkTable intakeNtTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("IntakeShooter");

    kickerOutputPub = intakeNtTab.getDoubleTopic("Kicker Output").publish();
    flywheelOutputPub = intakeNtTab.getDoubleTopic("Flywheel Output").publish();
  }

  public void setKickerPercentOutput(double percentOutput) {
    kickerMotor.set(percentOutput);
  }

  public void setFlywheelPercentOutput(double percentOutput) {
    flywheelMotor.set(percentOutput);
  }

  public void updateShuffleboard() {
    kickerOutputPub.set(kickerMotor.get());
    flywheelOutputPub.set(flywheelMotor.get());
  }

  @Override
  public void periodic() {
    updateShuffleboard();
  }
}
