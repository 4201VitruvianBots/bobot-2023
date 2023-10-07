package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BASE;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;

public class Wrist extends SubsystemBase {

  private double m_desiredSetpointRadians;
  private CONTROL_MODE m_controlMode = CONTROL_MODE.OPEN_LOOP;
  private final CANSparkMax m_wristMotor = new CANSparkMax(CAN.wristMotor, MotorType.kBrushless);
  private boolean m_userSetpoint;
  private double m_joystickInput;

  private PIDController m_pidController = new PIDController(INTAKE.kWristP, INTAKE.kWristI, INTAKE.kWristD);

  private ArmFeedforward m_feedForward =
      new ArmFeedforward(INTAKE.kWristS, INTAKE.kWristG, INTAKE.kWristV, INTAKE.kWristA);

  private DoublePublisher m_percentOutputPub,
    m_voltagePub,
    m_angleDegreesPub,
    m_encoderPositionPub,
    m_desiredAnglePub;
  private StringPublisher m_controlModePub;

  public Wrist() {
    m_wristMotor.setInverted(false);
    m_wristMotor.getEncoder().setPosition(0);

    NetworkTable wristNtTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Wrist");
    m_percentOutputPub = wristNtTab.getDoubleTopic("Wrist Percent Output").publish();
    m_voltagePub = wristNtTab.getDoubleTopic("Wrist Output Voltage").publish();
    m_angleDegreesPub = wristNtTab.getDoubleTopic("Wrist Angle Degrees").publish();
    m_encoderPositionPub = wristNtTab.getDoubleTopic("Wrist Encoder Units").publish();
    m_desiredAnglePub = wristNtTab.getDoubleTopic("Wrist Desired Angle").publish();
    m_controlModePub = wristNtTab.getStringTopic("Wrist Control Mode").publish();
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

  public void setSensorPosition(double position) {
    m_wristMotor.getEncoder().setPosition(position);
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
    return getSensorPosition() * BASE.CONSTANTS.kNeoEncoderUnitsToDegrees;
  }

  private double getSensorPosition() {
    return m_wristMotor.getEncoder().getPosition();
  }

  public void updateShuffleboard() {
    m_angleDegreesPub.set(getPositionDegrees());
    m_encoderPositionPub.set(m_wristMotor.getEncoder().getPosition());
    m_desiredAnglePub.set(Units.radiansToDegrees(m_desiredSetpointRadians));
    m_controlModePub.set(m_controlMode.name());

    if (RobotBase.isReal()) {
      m_percentOutputPub.set(m_wristMotor.get());
      m_voltagePub.set(m_wristMotor.getAppliedOutput());
    }
  }

  @Override
  public void periodic() {
    updateShuffleboard();

    switch (m_controlMode) {
      default:
      case OPEN_LOOP:
        double percentOutput = m_joystickInput * BASE.CONSTANTS.kPercentOutputMultiplier;

        setWristPercentOutput(percentOutput);

        if (RobotBase.isSimulation()) {
          m_voltagePub.set(percentOutput);
        }
        break;
      case CLOSED_LOOP:

        // FRC 5712 logic
        double feedForward = m_feedForward.calculate(m_desiredSetpointRadians, 0);
        double pid = m_pidController.calculate(getSensorPosition(), Units.radiansToDegrees(m_desiredSetpointRadians) / BASE.CONSTANTS.kNeoEncoderUnitsToDegrees);

        double output = feedForward + pid;

        m_wristMotor.setVoltage(output);

        if (RobotBase.isSimulation()) {
          m_voltagePub.set(output);
        }

        break;
    }
  }
}
