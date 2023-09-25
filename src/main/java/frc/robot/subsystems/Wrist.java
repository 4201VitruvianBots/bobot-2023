package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.CAN;

public class Wrist extends SubsystemBase {
  private double m_wristDesiredSetpointRadians;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  private final TalonFX wrist = new TalonFX(CAN.wristMotor);
  private boolean m_userSetpoint;

  public void SetWristDesiredSetpoint(SETPOINT desiredSetpoint) {
    m_wristDesiredSetpointRadians = desiredSetpoint.getWristSetpointRadians();
  }

  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }
}
