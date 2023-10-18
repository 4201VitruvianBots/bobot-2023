// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.CtreUtils.configureTalonFx;
import static frc.robot.utils.ModuleMap.MODULE_POSITION;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.SWERVE.MODULE;
import frc.robot.simulation.MotorSim;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;

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

  private TalonFXSimState m_turnMotorSimState;
  private TalonFXSimState m_driveMotorSimState;
  private CANcoderSimState m_angleEncoderSimState;

//  private DCMotorSim m_turnMotorModel =
//      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, .001);
//
//  private DCMotorSim m_driveMotorModel =
//      new DCMotorSim(
//          // Sim Values
//          MODULE.kDriveGearbox, MODULE.kDriveMotorGearRatio, 0.2);

  private DCMotorSim m_turnMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), MODULE.kTurnMotorGearRatio, 0.5);
  private MotorSim m_driveMotorSim;

  private double m_lastTime;

  Mechanism2d moduleVisualizer = new Mechanism2d(Units.inchesToMeters(6), Units.inchesToMeters(6));
  MechanismRoot2d moduleRoot;
  MechanismLigament2d moduleLigament;

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
    turnMotorConfig.Feedback.SensorToMechanismRatio = MODULE.kTurnMotorGearRatio;
    configureTalonFx(m_turnMotor, turnMotorConfig);

    var driveMotorConfig = CtreUtils.generateDriveMotorConfig();
    driveMotorConfig.Feedback.SensorToMechanismRatio = MODULE.kDriveMotorGearRatio;
    configureTalonFx(m_driveMotor, driveMotorConfig);

    m_lastAngle = getHeadingDegrees();

    initSmartDashboard();

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());

    if (!RobotBase.isReal()) {
      m_turnMotorSimState = m_turnMotor.getSimState();
      m_driveMotorSimState = m_driveMotor.getSimState();
      m_angleEncoderSimState = m_angleEncoder.getSimState();

//      m_turnMotorSim = MotorSim.createSimpleMotor(
//              m_turnMotor.getDeviceID(),
//              "Falcon 500",
//              0,
//              0.135872794,
//              0.002802309,
//              0,
//              0,
//              DCMotor.getFalcon500(1)
//      );
//
      m_driveMotorSim = MotorSim.createSimpleMotor(
              m_driveMotor.getDeviceID(),
              "Falcon 500",
              0,
              0.134648227,
              0.002802309,
//              2.46330,
//          0.12872,
              0,
              0,
              DCMotor.getFalcon500(1)
      );

      angleOffset = 0;
    }
    m_angleOffset = angleOffset;
    initModuleHeading();

    moduleRoot =
            moduleVisualizer.getRoot(
                    "ModuleCenter_" + m_modulePosition.ordinal(),
                    Units.inchesToMeters(3),
                    Units.inchesToMeters(3));
    moduleLigament =
            moduleRoot.append(
                    new MechanismLigament2d(
                            "ModuleDirection_" + m_modulePosition.ordinal(),
                            (getVelocityMetersPerSecond() / SWERVE.DRIVE.kMaxSpeedMetersPerSecond) * .75
                                    + .25,
                            getHeadingDegrees()));

    SmartDashboard.putData("SwerveModule2D_" + m_modulePosition.ordinal(), moduleVisualizer);
  }

  private void initModuleHeading() {
    var encoderConfig = CtreUtils.generateCanCoderConfig();
    encoderConfig.MagnetSensor.MagnetOffset = m_angleOffset / 360.0;
    m_angleEncoder.getConfigurator().apply(encoderConfig);
    if (RobotBase.isReal()) Timer.delay(0.2);
    m_angleEncoder.optimizeBusUtilization(255);
    resetAngleToAbsolute();

    // Check if the offset was applied properly. Delay to give it some time to set
    if (RobotBase.isReal()) {
      Timer.delay(0.1);
      m_initSuccess =
          Math.abs(getHeadingDegrees() + m_angleOffset - m_angleEncoder.getAbsolutePosition().getValue())
              < 1.0;
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
      //  double newAngle = getHeadingDegrees() - m_angleOffset + angle;
    m_turnMotor.setPosition(m_angleEncoder.getAbsolutePosition().getValue());
  }

  public double getHeadingDegrees() {
    return 360 * m_turnMotor.getPosition().getValue();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    return m_driveMotor.getVelocity().getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public double getDriveMeters() {
    return m_driveMotor.getPosition().getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    m_desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = m_desiredState.speedMetersPerSecond / DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.setControl(driveMotorDutyControl.withOutput(percentOutput));
    } else {
      double velocityRPM =
          m_desiredState.speedMetersPerSecond / (MODULE.kWheelDiameterMeters * Math.PI);
      m_driveMotor.setControl(
          driveMotorVelocityControl
              .withVelocity(velocityRPM)
              .withFeedForward(feedforward.calculate(velocityRPM)));
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
            m_angleEncoder.getAbsolutePosition().getValue() * 360.0);
    Logger.getInstance()
        .recordOutput("Swerve/Module[" + m_moduleNumber + "] Angle Offset", m_angleOffset);
    Logger.getInstance()
        .recordOutput("Swerve/Module[" + m_moduleNumber + "] Motor Heading", getHeadingDegrees());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();

    moduleLigament.setLength(
            ((getVelocityMetersPerSecond() / SWERVE.DRIVE.kMaxSpeedMetersPerSecond) * .75) + .25);
    moduleLigament.setAngle(getHeadingDegrees());
  }

  public void simulationInit() {
    m_lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                    m_turnMotorSim.getCurrentDrawAmps(), m_driveMotorSim.getCurrentDrawAmps()));

    m_turnMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    ConduitApi.getInstance().captureData();
    LoggedDriverStation.getInstance().periodic();
    LoggedSystemStats.getInstance().periodic();

    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));
    m_driveMotorSim.setInputVoltage(
            MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));

    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - m_lastTime;
    m_turnMotorSim.update(dt);
//    m_driveMotorSim.update(dt);
    m_driveMotorSim.run();

//    var turnVelocityRps = m_turnMotorSim.getAngularVelocityRPM() / 60.0;
//    var turnDistance = turnVelocityRps * dt;
//    var driveVelocityRps = m_driveMotorSim.getAngularVelocityRPM() * MODULE.kDriveMotorGearRatio / 60.0;
//    var driveDistance = driveVelocityRps * dt;
//
//    m_turnMotorSimState.addRotorPosition(turnDistance * MODULE.kTurnMotorGearRatio);
//    m_turnMotorSimState.setRotorVelocity(turnVelocityRps * MODULE.kTurnMotorGearRatio);
//    m_angleEncoderSimState.addPosition(turnDistance);
//    m_angleEncoderSimState.setVelocity(turnVelocityRps);
//    m_driveMotorSimState.setRawRotorPosition(driveDistance);
//    m_driveMotorSimState.setRotorVelocity(driveVelocityRps);

    var turnVelocityRps = m_turnMotorSim.getAngularVelocityRPM() / 60.0;
    var turnDistance = turnVelocityRps * dt;
    var driveVelocityRps = m_driveMotorSim.getAngularVelocityRPM() * MODULE.kDriveMotorGearRatio / 60.0;
    var driveDistance = driveVelocityRps * dt;

    m_turnMotorSimState.addRotorPosition(turnDistance * MODULE.kTurnMotorGearRatio);
    m_turnMotorSimState.setRotorVelocity(turnVelocityRps * MODULE.kTurnMotorGearRatio);
    m_angleEncoderSimState.addPosition(turnDistance);
    m_angleEncoderSimState.setVelocity(turnVelocityRps);
    m_driveMotorSimState.addRotorPosition(driveDistance);
    m_driveMotorSimState.setRotorVelocity(driveVelocityRps);

    m_lastTime = currentTime;
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turnMotor.close();
    m_angleEncoder.close();
  }
}
