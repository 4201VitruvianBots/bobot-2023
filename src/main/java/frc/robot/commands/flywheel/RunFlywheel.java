// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.INTAKE.FLYWHEEL_SPEED;
import frc.robot.subsystems.IntakeShooter;

public class RunFlywheel extends CommandBase {

  IntakeShooter m_intakeShooter;
  FLYWHEEL_SPEED m_speed;

  /** Creates a new RunIntake. */
  public RunFlywheel(IntakeShooter intakeShooter, FLYWHEEL_SPEED speed) {
    m_intakeShooter = intakeShooter;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setFlywheelPercentOutput(m_speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_intakeShooter.setFlywheelPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
