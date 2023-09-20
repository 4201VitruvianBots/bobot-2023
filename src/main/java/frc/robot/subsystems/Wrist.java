package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CONTROL_MODE;

public class Wrist extends SubsystemBase {
  private double m_wristDesiredSetpointRadians;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  private final TalonFX wrist = new TalonFX(CAN.wristMotor);
  private boolean m_userSetpoint;

  public void SetWristDesiredSetpoint(Constants.SETPOINT desiredSetpoint) {
    m_wristDesiredSetpointRadians = desiredSetpoint.getWristSetpointRadians();
  }

  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }
}
