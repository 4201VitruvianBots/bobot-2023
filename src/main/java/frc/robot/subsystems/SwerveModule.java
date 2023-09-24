// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.SWERVE_MODULE_POSITION;
import frc.robot.Constants.SWERVE_MODULE;
import frc.robot.utils.CtreUtils;

public class SwerveModule extends SubsystemBase implements AutoCloseable {
  private final SWERVE_MODULE_POSITION m_modulePosition;
  private final int m_moduleNumber;
  private final TalonFX m_turnMotor;
  private final DutyCycleOut driveMotorDutyControl = new DutyCycleOut(0);
  private final VelocityVoltage driveMotorVelocityControl = new VelocityVoltage(0);
  private final TalonFX m_driveMotor;
  private final PositionVoltage turnMotorPositionControl = new PositionVoltage(0);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();
  private final CANcoder m_angleEncoder;
  private final double m_angleOffset;
  private double m_lastAngle;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SWERVE_MODULE.ksDriveVoltSecondsPerMeter,
          SWERVE_MODULE.kvDriveVoltSecondsSquaredPerMeter,
          SWERVE_MODULE.kaDriveVoltSecondsSquaredPerMeter);
  // Logging setup

  private final DoubleLogEntry moduleTurnCurrentEntry;
  private final DoubleLogEntry moduleDriveCurrentEntry;

  private DoublePublisher moduleMotorHeadingPub, moduleEncoderHeadingPub;
  private BooleanPublisher moduleEncoderHealthPub;
  private PositionVoltage turnMotorVoltage = new PositionVoltage(0).withSlot(0);

  public SwerveModule(
      SWERVE_MODULE_POSITION modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANcoder angleEncoder,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = angleOffset;

    initModuleHeading();

    var turnMotorConfig = CtreUtils.generateTurnMotorConfig();
    var driveMotorConfig = CtreUtils.generateDriveMotorConfig();
    turnMotorConfig.Feedback.RotorToSensorRatio = Constants.SWERVE_MODULE.kTurningMotorGearRatio;

    m_turnMotor.getConfigurator().apply(turnMotorConfig);
    m_driveMotor.getConfigurator().apply(driveMotorConfig);
    m_turnMotor.setControl(turnMotorVoltage.withPosition(0));
    //    m_turnMotor.setSelectedSensorPosition(0);
    // m_turnMotor.setPosition(0);
    // m_angleEncoder.configMagnetOffset(m_angleOffset);
    m_lastAngle = getHeadingDegrees();

    initSmartDashboard();
    DataLog m_log = DataLogManager.getLog();
    moduleTurnCurrentEntry =
        new DoubleLogEntry(m_log, "/swerve/" + m_modulePosition.name() + "/turnCurrent");
    moduleDriveCurrentEntry =
        new DoubleLogEntry(m_log, "/swerve/" + m_modulePosition.name() + "/driveCurrent");

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());
  }

  private void initModuleHeading() {
    if (RobotBase.isReal()) Timer.delay(0.2);
    m_angleEncoder.getConfigurator().apply(CtreUtils.generateCanCoderConfig());
    //    m_angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    //    m_angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
    resetAngleToAbsolute();

    // Check if the offset was applied properly. Delay to give it some time to set
    if (RobotBase.isReal()) {
      Timer.delay(0.1);
      m_initSuccess =
          Math.abs(
                  getHeadingDegrees()
                      + m_angleOffset
                      - (360 * m_angleEncoder.getAbsolutePosition().getValue()))
              < 1.0;
    } else m_initSuccess = true;
  }

  public boolean getInitSuccess() {
    return m_initSuccess;
  }

  public SWERVE_MODULE_POSITION getModulePosition() {
    return m_modulePosition;
  }

  public void resetAngleToAbsolute() {
    resetAngle(0);
  }

  public void resetAngle(double angle) {
    double newAngle =
        (360.0 * m_angleEncoder.getAbsolutePosition().getValue()) - m_angleOffset + angle;
    m_turnMotor.setRotorPosition(newAngle / 360.0);
  }

  public double getHeadingDegrees() {
    return 360 * m_turnMotor.getRotorPosition().getValue();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    return m_driveMotor.getRotorVelocity().getValue()
        * SWERVE_MODULE.kWheelDiameterMeters
        * Math.PI;
  }

  public double getDriveMeters() {
    return m_driveMotor.getRotorPosition().getValue()
        * SWERVE_MODULE.kWheelDiameterMeters
        * Math.PI;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / SWERVE_DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.setControl(driveMotorDutyControl.withOutput(percentOutput));
    } else {
      double velocity =
          desiredState.speedMetersPerSecond / (SWERVE_MODULE.kWheelDiameterMeters * Math.PI);
      m_driveMotor.setControl(
          driveMotorVelocityControl
              .withVelocity(velocity)
              .withFeedForward(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.setControl(turnMotorPositionControl.withPosition(angle / 360.0));
    m_lastAngle = angle;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getHeadingRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }

  public void setDriveBrake() {
    m_driveMotor.setControl(brakeControl);
  }

  public void setDriveNeutral() {
    m_driveMotor.setControl(neutralControl);
  }

  public void setTurnBrake() {
    m_turnMotor.setControl(brakeControl);
  }

  public void setTurnNeutral() {
    m_turnMotor.setControl(neutralControl);
  }

  private void initSmartDashboard() {
    var moduleTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Swerve");
    moduleEncoderHeadingPub =
        moduleTab.getDoubleTopic("Module (" + m_moduleNumber + ") Encoder Heading").publish();
    moduleTab
        .getDoubleTopic("Module (" + m_moduleNumber + ") Encoder Offset")
        .publish()
        .set(m_angleOffset);
    moduleEncoderHealthPub =
        moduleTab.getBooleanTopic("Module (" + m_moduleNumber + ") Encoder Health").publish();
    moduleMotorHeadingPub =
        moduleTab.getDoubleTopic("Module (" + m_moduleNumber + ") Motor Heading").publish();
  }

  private void updateSmartDashboard() {
    moduleMotorHeadingPub.set(getHeadingDegrees());
    moduleEncoderHealthPub.set(getInitSuccess());
    moduleEncoderHeadingPub.set(m_angleEncoder.getAbsolutePosition().getValue() * 360);
  }

  public void updateLog() {
    moduleTurnCurrentEntry.append(m_turnMotor.getSupplyVoltage().getValue());
    moduleDriveCurrentEntry.append(m_driveMotor.getSupplyVoltage().getValue());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    //    updateLog();
  }

  @Override
  public void simulationPeriodic() {}

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
