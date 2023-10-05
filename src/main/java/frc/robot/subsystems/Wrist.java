package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BASE;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;

public class Wrist extends SubsystemBase {

  private double m_desiredSetpointRadians;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  private final CANSparkMax m_wristMotor = new CANSparkMax(CAN.wristMotor, MotorType.kBrushless);
  private boolean m_userSetpoint;
  private double m_joystickInput;

  private PIDController m_controller =
      new PIDController(INTAKE.kWristP, INTAKE.kWristI, INTAKE.kWristD);

  public Wrist() {
    m_wristMotor.setInverted(true);
  }

  public CONTROL_MODE getClosedLoopControlMode() {
    return m_controlMode;
  }

  public void SetWristDesiredSetpoint(SETPOINT desiredSetpoint) {
    m_desiredSetpointRadians = desiredSetpoint.getWristSetpointRadians();
  }

  public void setUserInput(double input) {
    m_joystickInput = input;
  }

  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }
  // setSelectedSensorPosition(angleDegrees / BASE.CONSTANTS.encoderUnitsToDegrees);

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }
  // public void resetAngleDegrees(double angleDegrees) {
  // m_wristMotor.set

  public void setWristPercentOutput(double output) {
    m_wristMotor.set(output);
  }

  public void setSetpointPositionRadians(double desiredAngleRadians) {
    m_desiredSetpointRadians = desiredAngleRadians;
  }

  public double getPositionRadians() {
    return Units.degreesToRadians(getPositionDegrees());
  }

  public double getPositionDegrees() {
    return getSensorPosition() * BASE.CONSTANTS.encoderUnitsToDegrees;
  }

  private double getSensorPosition() {
    return m_wristMotor.getEncoder().getPosition();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Wrist Angles Degrees", getPositionDegrees());
  }

  @Override
  public void periodic() {

    updateSmartDashboard();
    switch (m_controlMode) {
      default:
      case OPEN_LOOP:
        double percentOutput = m_joystickInput * BASE.CONSTANTS.kPercentOutputMultiplier;

      case CLOSED_LOOP:
        setWristPercentOutput(
            m_controller.calculate(
                m_wristMotor.getEncoder().getPosition(),
                m_desiredSetpointRadians * INTAKE.kEncoderUnitsPerRotation));

        break;
    }
  }
}
