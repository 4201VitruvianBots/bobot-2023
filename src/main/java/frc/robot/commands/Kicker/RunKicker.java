// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Kicker;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeShooter;

public class RunKicker extends CommandBase {

  IntakeShooter m_intakeShooter;
  /** Creates a new RunIntake. */
  public RunKicker(IntakeShooter intakeShooter) {
    m_intakeShooter = intakeShooter;

    addRequirements(m_intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setIntakePercentOutput(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_intakeShooter.setIntakePercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
