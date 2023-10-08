// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.BASE.XBOX_BUTTON;
import frc.robot.subsystems.Wrist;

public class WristHandler extends CommandBase {
  private Wrist m_wrist;
  private SETPOINT m_desiredState;
  private DoubleSupplier m_joystickY;
  private XBOX_BUTTON m_buttonPressed;

  public WristHandler(Wrist wrist, SETPOINT desiredState, DoubleSupplier joystickY, XBOX_BUTTON buttonPressed) {
    m_wrist = wrist;
    m_desiredState = desiredState;
    m_joystickY = joystickY;
    m_buttonPressed = buttonPressed;

    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLog log = DataLogManager.getLog();
    StringLogEntry buttonEntry = new StringLogEntry(log, "/wrist/button");
    buttonEntry.append(m_buttonPressed.name());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_desiredState != null) {
      m_wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
      m_wrist.setUserSetpoint(true);
      m_wrist.SetWristDesiredSetpoint(m_desiredState);
    } else {
      m_wrist.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
      m_wrist.setUserSetpoint(false);
      m_wrist.setUserInput(MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.05));
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
