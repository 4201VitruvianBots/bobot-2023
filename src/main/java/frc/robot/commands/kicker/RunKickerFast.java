// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kicker;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeShooter;

public class RunKickerFast extends CommandBase {

  IntakeShooter m_intakeShooter;
  /** Creates a new RunIntake. */
  public RunKickerFast(IntakeShooter intakeShooter) {
    m_intakeShooter = intakeShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setKickerPercentOutput(-0.85);
    m_intakeShooter.setFlywheelPercentOutput(0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_intakeShooter.setKickerPercentOutput(0);
    m_intakeShooter.setFlywheelPercentOutput(0);
  }
  // 236 (Bent's favorite team)
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
