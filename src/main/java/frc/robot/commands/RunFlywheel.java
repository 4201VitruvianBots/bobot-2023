// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeShooter;

public class RunFlywheel extends CommandBase {

  IntakeShooter m_intakeShooter;
  /** Creates a new RunIntake. */
  public RunFlywheel(IntakeShooter intakeShooter) {
    m_intakeShooter = intakeShooter;

    addRequirements(m_intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setFlywheelPercentOutput(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    m_intakeShooter.setFlywheelPercentOutput(0);

  }

  public enum SETPOINT {
    // Units are in Radians
    STOWED(Units.degreesToRadians(98.0)),
    INTAKING_LOW_CUBE(Units.degreesToRadians(-13.5)),
    INTAKING_LOW_CONE(Units.degreesToRadians(16)),
    SCORE_LOW_REVERSE(Units.degreesToRadians(-14.0)),
    SCORE_LOW_CONE(Units.degreesToRadians(120.0)),
    SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
    SCORE_MID_CONE(Units.degreesToRadians(150.5)),
    SCORE_MID_CUBE(Units.degreesToRadians(132.0)),
    SCORE_HIGH_CONE(Units.degreesToRadians(151.5)),
    SCORE_HIGH_CUBE(Units.degreesToRadians(147.0)),
    INTAKING_EXTENDED_CONE(Units.degreesToRadians(121.3)),
    INTAKING_EXTENDED_CUBE(SCORE_HIGH_CUBE.get());

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
