// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SetArcadeDrive extends CommandBase {
  /** Creates a new SetArcadeDrive. */

  private DriveTrain m_driveTrain;
  private DoubleSupplier m_throttle, m_turn;
  private double joystickX, joystickY, throttle;

  public SetArcadeDrive(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_throttle = throttle;
    m_turn = turn;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    joystickY = m_throttle.getAsDouble();
    // joystickX = MathUtil.applyDeadband(m_turn.getAsDouble(), 0.05);

    m_driveTrain.setMotorArcadeDrive(joystickY);
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
