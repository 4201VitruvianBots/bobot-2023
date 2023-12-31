// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.simulation.SimConstants.usePhysicsSim;
import static frc.robot.utils.CtreUtils.configureCANCoder;
import static frc.robot.utils.CtreUtils.configureTalonFx;
import static frc.robot.utils.ModuleMap.MODULE_POSITION;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.SWERVE.MODULE;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase implements AutoCloseable {
  private final ModuleMap.MODULE_POSITION m_modulePosition;
  private final int m_moduleNumber;
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;

  private final double m_angleOffset;
  private double m_lastAngle;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;
  private SwerveModuleState m_desiredState;

  private final DutyCycleOut driveMotorDutyControl = new DutyCycleOut(0);
  private final VelocityVoltage driveMotorVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage turnPositionControl = new PositionVoltage(0);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          MODULE.ksDriveVoltSecondsPerMeter,
          MODULE.kvDriveVoltSecondsSquaredPerMeter,
          MODULE.kaDriveVoltSecondsSquaredPerMeter);

  private TalonFXSimState m_turnMotorSim;
  private TalonFXSimState m_driveMotorSim;

  private final FlywheelSim m_turnMotorModel =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.25, 0.000001),
          MODULE.kTurnGearbox,
          MODULE.kTurningMotorGearRatio,
          VecBuilder.fill(0));

  private final FlywheelSim m_driveMotorModel =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.8, 0.6),
          MODULE.kDriveGearbox,
          MODULE.kDriveMotorGearRatio);

  private double m_turnMotorSimDistance;
  private double m_driveMotorSimDistance;
  private double m_turnSimInput;
  private double m_driveSimInput;
  private double m_lastTime;

  public SwerveModule(
      MODULE_POSITION modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANcoder angleEncoder,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;

    var turnMotorConfig = CtreUtils.generateTurnMotorConfig();
    turnMotorConfig.Feedback.RotorToSensorRatio = MODULE.kTurningMotorGearRatio;
    configureTalonFx(m_turnMotor, turnMotorConfig);

    var driveMotorConfig = CtreUtils.generateDriveMotorConfig();
    driveMotorConfig.Feedback.RotorToSensorRatio = MODULE.kDriveMotorGearRatio;
    configureTalonFx(m_driveMotor, turnMotorConfig);

    m_turnMotor.setControl(turnPositionControl.withPosition(0));
    //    m_turnMotor.setSelectedSensorPosition(0);
    // m_turnMotor.setPosition(0);
    // m_angleEncoder.configMagnetOffset(m_angleOffset);
    m_lastAngle = getHeadingDegrees();

    initSmartDashboard();

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());

    if (!RobotBase.isReal()) {
      m_turnMotorSim = m_turnMotor.getSimState();
      m_driveMotorSim = m_driveMotor.getSimState();
      angleOffset = 0;
    }
    m_angleOffset = angleOffset;
    initModuleHeading();
  }

  private void initModuleHeading() {
    var encoderConfig = CtreUtils.generateCanCoderConfig();
    encoderConfig.MagnetSensor.MagnetOffset = m_angleOffset;
    configureCANCoder(m_angleEncoder, encoderConfig);
    resetAngleToAbsolute();

    // Check if the offset was applied properly. Delay to give it some time to set
    if (RobotBase.isReal()) {
      Timer.delay(0.1);
      m_initSuccess = m_angleEncoder.getAbsolutePosition().getValue() < 1.0;
    } else m_initSuccess = true;
  }

  public boolean getInitSuccess() {
    return m_initSuccess;
  }

  public MODULE_POSITION getModulePosition() {
    return m_modulePosition;
  }

  public void resetAngleToAbsolute() {
    resetAngle(0);
  }

  public void resetAngle(double angle) {
//    double newAngle = getHeadingDegrees() - m_angleOffset + angle;
    m_turnMotor.setPosition(angle / 360.0);
  }

  public double getHeadingDegrees() {
    return 360 * m_turnMotor.getRotorPosition().getValue();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    return m_driveMotor.getRotorVelocity().getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public double getDriveMeters() {
    return m_driveMotor.getRotorPosition().getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    m_desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = m_desiredState.speedMetersPerSecond / DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.setControl(driveMotorDutyControl.withOutput(percentOutput));
    } else {
      double velocity =
          m_desiredState.speedMetersPerSecond / (MODULE.kWheelDiameterMeters * Math.PI);
      m_driveMotor.setControl(
          driveMotorVelocityControl
              .withVelocity(velocity)
              .withFeedForward(m_desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(m_desiredState.speedMetersPerSecond) <= (DRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : m_desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.setControl(turnPositionControl.withPosition(angle / 360.0));
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

  private void initSmartDashboard() {}

  private void updateSmartDashboard() {}

  public void updateLog() {
    Logger.getInstance()
        .recordOutput(
            "Swerve/Module[" + m_moduleNumber + "] Encoder Heading",
            m_angleEncoder.getAbsolutePosition().getValue());
    Logger.getInstance()
        .recordOutput("Swerve/Module[" + m_moduleNumber + "] Angle Offset", m_angleOffset);
    Logger.getInstance()
        .recordOutput("Swerve/Module[" + m_moduleNumber + "] Motor Heading", getHeadingDegrees());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();
  }

  @Override
  public void simulationPeriodic() {
    // TODO: Fix sim
    if (usePhysicsSim) {
      m_turnMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      m_driveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      m_turnMotorModel.setInputVoltage(MathUtil.clamp(m_turnMotorSim.getMotorVoltage(), -12, 12));
      m_driveMotorModel.setInputVoltage(MathUtil.clamp(m_driveMotorSim.getMotorVoltage(), -12, 12));

      double currentTime = Timer.getFPGATimestamp();
      double dt = currentTime - m_lastTime;
      m_turnMotorModel.update(dt);
      m_driveMotorModel.update(dt);

      m_turnMotorSimDistance += m_turnMotorModel.getAngularVelocityRadPerSec() * dt;
      m_driveMotorSimDistance += m_driveMotorModel.getAngularVelocityRadPerSec() * dt;

      Unmanaged.feedEnable(20);

      m_turnMotorSim.setRawRotorPosition(m_turnMotorSimDistance * MODULE.kTurningMotorGearRatio);
      m_driveMotorSim.setRawRotorPosition(m_driveMotorSimDistance * MODULE.kDriveMotorGearRatio);
      m_turnMotorSim.setRotorVelocity(m_turnMotorModel.getAngularVelocityRPM() / 60.0);
      m_driveMotorSim.setRotorVelocity(m_driveMotorModel.getAngularVelocityRPM() / 60.0);

      m_lastTime = currentTime;
    } else {
      //      double currentTime = Timer.getFPGATimestamp();
      //      double dt = currentTime - m_lastTime;
      //      double newDriveSpeed = m_desiredState.speedMetersPerSecond;
      //      m_driveMotorSimDistance += m_desiredState.speedMetersPerSecond * dt;
      //      double newRotationPosition = m_desiredState.angle.getRotations();
      //      m_turnMotorSimDistance
      //      m_ .setRotorVelocity(newDriveSpeed / (MODULE.kWheelDiameterMeters * Math.PI));
      //
      //      m_driveMotorSim.setRawRotorPosition(m_driveMotorSimDistance);
      //      m_driveMotorSim.setRotorVelocity(newDriveSpeed / (MODULE.kWheelDiameterMeters *
      // Math.PI));
      //      m_lastTime = currentTime;
    }
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turnMotor.close();
    m_angleEncoder.close();
  }
}
