// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.subsystems.Wrist;
import java.util.function.DoubleSupplier;

public class SetWristManual extends CommandBase {
  /** Creates a new SetWristManual. */
  Wrist m_wrist;

  DoubleSupplier m_output;

  public SetWristManual(Wrist wrist, DoubleSupplier output) {
    m_wrist = wrist;
    m_output = output;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Adds a Deadband so joystick Xs below 0.05 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_output.getAsDouble(), 0.1);

    if (joystickYDeadbandOutput > 0) {
      System.out.println("Running wrist joystick");
      m_wrist.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
      m_wrist.setUserSetpoint(true);
      m_wrist.setUserInput(joystickYDeadbandOutput);
    } else {
      m_wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
      m_wrist.setUserSetpoint(false);
    }
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
