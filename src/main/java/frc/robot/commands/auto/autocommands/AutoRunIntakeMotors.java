// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.INTAKE.FLYWHEEL_SPEED;
import frc.robot.constants.INTAKE.KICKER_SPEED;
import frc.robot.subsystems.IntakeShooter;

public class AutoRunIntakeMotors extends CommandBase {

  IntakeShooter m_intakeShooter;
  FLYWHEEL_SPEED m_flywheelspeed;
  KICKER_SPEED m_kickerspeed;

  /** Creates a new RunIntake. */
  public AutoRunIntakeMotors(
      IntakeShooter intakeShooter, FLYWHEEL_SPEED flywheelspeed, KICKER_SPEED kickerspeed) {
    m_intakeShooter = intakeShooter;
    m_flywheelspeed = flywheelspeed;
    m_kickerspeed = kickerspeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setFlywheelPercentOutput(m_flywheelspeed.get());

    m_intakeShooter.setKickerPercentOutput(m_kickerspeed.get());
    // TODO: set wrist to setpoitns
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
