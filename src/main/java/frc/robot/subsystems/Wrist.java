package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;

public class Wrist extends SubsystemBase {
  private double m_desiredSetpointRadians;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  private final CANSparkMax m_wristMotor = new CANSparkMax(CAN.wristMotor, MotorType.kBrushless);
  private boolean m_userSetpoint;

  private PIDController m_controller =
      new PIDController(INTAKE.kWristP, INTAKE.kWristI, INTAKE.kWristD);

  public void SetWristDesiredSetpoint(SETPOINT desiredSetpoint) {
    m_desiredSetpointRadians = desiredSetpoint.getWristSetpointRadians();
  }

  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  public void setWristPercentOutput(double output) {
    m_wristMotor.set(output);
  }

  @Override
  public void periodic() {

    switch (m_controlMode) {
      default:
      case OPEN_LOOP:

      case CLOSED_LOOP:
        setWristPercentOutput(
            m_controller.calculate(
                m_wristMotor.getEncoder().getPosition(), m_desiredSetpointRadians));

        break;
    }
  }
}
