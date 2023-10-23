// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VISION.CAMERA_SERVER;
import java.util.stream.DoubleStream;

public class Vision extends SubsystemBase implements AutoCloseable {
  private final SwerveDrive m_swerveDrive;
  private final Controls m_controls;

  // Mech2d setup
  private MechanismLigament2d m_limelightLigament2d;

  private final NetworkTable m_limelight;

  private final DoubleLogEntry limelightTargetValid;
  private final DoubleLogEntry leftLocalizerTargetValid;

  private final Timer searchPipelineTimer = new Timer();
  private final double searchPipelineWindow = 0.4;

  private final Timer searchTimer = new Timer();

  private double startTime, timestamp;
  private boolean timerStart;
  private double m_pipeline;

  private final Pose2d defaultPose = new Pose2d(-5, -5, new Rotation2d());

  private final double[] defaultDoubleArray = {0, 0, 0, 0, 0, 0, 0};

  private final int[] tagIds = new int[10];
  double[] robotPosX = new double[10];
  double[] robotPosY = new double[10];
  double[] robotPosYaw = new double[10];

  double[] tagPosX = new double[10];
  double[] tagPosY = new double[10];

  public Vision(SwerveDrive swerveDrive, DataLog logger, Controls controls) {
    m_swerveDrive = swerveDrive;
    m_controls = controls;

    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

    PortForwarder.add(5800, CAMERA_SERVER.LIMELIGHT.toString(), 5800);
    PortForwarder.add(5801, CAMERA_SERVER.LIMELIGHT.toString(), 5801);
    PortForwarder.add(5802, CAMERA_SERVER.LIMELIGHT.toString(), 5802);
    PortForwarder.add(5803, CAMERA_SERVER.LIMELIGHT.toString(), 5803);
    PortForwarder.add(5804, CAMERA_SERVER.LIMELIGHT.toString(), 5804);
    PortForwarder.add(5805, CAMERA_SERVER.LIMELIGHT.toString(), 5805);

    limelightTargetValid = new DoubleLogEntry(logger, "/vision/limelight_tv");
    leftLocalizerTargetValid = new DoubleLogEntry(logger, "/vision/fLocalizer_tv");

    resetSearch();
    resetPipelineSearch();
    initSmartDashboard();
  }

  public MechanismLigament2d getLimelightLigament() {
    return m_limelightLigament2d;
  }

  /**
   * Given a camera, return a boolean value based on if it sees a target or not.
   *
   * @return true: Camera has a target. false: Camera does not have a target
   */
  public boolean getValidTarget(CAMERA_SERVER location) {
    return getValidTargetType(location) > 0;
  }

  /*
   * Whether the limelight has any valid targets (0 or 1)
   */
  public double getValidTargetType(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("tv").getDouble(0);
      default:
        return 0;
    }
  }

  public double[] getAprilTagIds(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("tid").getDoubleArray(defaultDoubleArray);
      default:
        return defaultDoubleArray;
    }
  }

  /*
   * Horizontal Offset From Crosshair To Target
   */
  public double getTargetXAngle(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return -m_limelight.getEntry("tx").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Vertical Offset From Crosshair To Target
   */
  public double getTargetYAngle(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("ty").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
   */
  public double getCameraLatency(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("tl").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Target Area (0% of image to 100% of image)
   */
  public double getTargetArea(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("ta").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Full JSON dump of targeting results
   */
  public double getJSON(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("json").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Pipeline 1 = cube
   * Pipeline 2 = cone
   */
  public void setPipeline(CAMERA_SERVER location, double pipeline) {
    m_pipeline = pipeline;
  }

  public void updatePipeline() {
    m_limelight.getEntry("pipeline").setDouble(1);
  }

  public double getPipeline(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("pipeline").getDouble(0);
      default:
        return 0.0;
    }
  }

  // false == cube, true == cone
  public boolean getGamePieceType(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        if (getPipeline(location) == 2.0 && searchLimelightTarget(location)) {
          return true; // cone
        } else if (getPipeline(location) == 1.0 && searchLimelightTarget(location)) {
          return false; // cube
        }
      default:
        return false;
    }
  }

  /*
   * resets timer for pipeline reconnection
   */
  public void resetSearch() {
    searchTimer.reset();
    searchTimer.start();
  }

  /*
   * resets timer for pipeline finder
   */
  public void resetPipelineSearch() {
    searchPipelineTimer.reset();
    searchPipelineTimer.start();
  }

  /*
   * Look for any target
   */
  public boolean searchLimelightTarget(CAMERA_SERVER location) {
    //    if (getPipeline(location) == 1.0
    //        && m_intakeSub.getIntakeCubeState()) { // CUBE and if we're looking for cube
    //      return getValidTargetType(location) == 1.0
    //          && getTargetArea(location) > 1.0; // target read within threshold
    //    } else if (getPipeline(location) == 2.0
    //        && m_intakeSub.getIntakeConeState()) { // CONE and if we're looking for cone
    //      return getValidTargetType(location) == 1.0
    //          && getTargetArea(location) > 1.0; // target read within threshold
    //    }
    return false;
  }

  /*
   * Collects transformation/rotation data from limelight
   */
  public double[] getBotPose(CAMERA_SERVER location) {
    DriverStation.Alliance allianceColor = Controls.getAllianceColor();
    double[] botPose = new double[0];
    switch (location) {
      case LIMELIGHT:
        botPose = m_limelight.getEntry("botpose").getDoubleArray(defaultDoubleArray);
        break;
        //      case RIGHT_LOCALIZER:
        //        var rawBotPose = defaultDoubleArray;
        //        switch (allianceColor) {
        //          case Red:
        //            rawBotPose =
        //
        // m_rightLocalizer.getEntry("botpose_wpired").getDoubleArray(defaultDoubleArray);
        //            break;
        //          case Blue:
        //            rawBotPose =
        //
        // m_rightLocalizer.getEntry("botpose_wpiblue").getDoubleArray(defaultDoubleArray);
        //            break;
        //          default:
        //            rawBotPose =
        // m_rightLocalizer.getEntry("botpose").getDoubleArray(defaultDoubleArray);
        //
        //            if (rawBotPose.length > 0) {
        //              rawBotPose[0] = 15.980 / 2 + rawBotPose[0];
        //              rawBotPose[1] = 8.210 / 2 + rawBotPose[1];
        //              return rawBotPose;
        //            }
        //            break;
        //        }
    }
    if (botPose.length > 0) return botPose;
    else return defaultDoubleArray;
  }

  /**
   * Get the timestamp of the detection results.
   *
   * @return Robot Pose in meters
   */
  public double getDetectionTimestamp(CAMERA_SERVER location) {
    switch (location) {
      case LIMELIGHT:
        return m_limelight.getEntry("timestamp").getDouble(0);
      default:
        return 0;
    }
  }

  public Pose2d getRobotPose2d(CAMERA_SERVER location) {
    double[] pose = getBotPose(location);
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
  }

  public Pose2d[] getRobotPoses2d(CAMERA_SERVER location) {
    Pose2d[] poseArray = {defaultPose};

    if (getValidTarget(location)) {
      robotPosX = m_limelight.getEntry("Robot Pose X").getDoubleArray(new double[] {});
      robotPosY = m_limelight.getEntry("Robot Pose Y").getDoubleArray(new double[] {});
      robotPosYaw = m_limelight.getEntry("Robot Pose Yaw").getDoubleArray(new double[] {});
      poseArray = new Pose2d[robotPosX.length];
      for (int i = 0; i < robotPosX.length; i++)
        poseArray[i] =
            new Pose2d(robotPosX[i], robotPosY[i], Rotation2d.fromDegrees(robotPosYaw[i]));
    }

    return poseArray;
  }

  public Pose2d[] getTagPoses2d(CAMERA_SERVER location) {
    Pose2d[] poseArray = {defaultPose};

    if (getValidTarget(location)) {
      try {
        tagPosX = m_limelight.getEntry("Tag Pose X").getDoubleArray(new double[] {});
        tagPosY = m_limelight.getEntry("Tag Pose Y").getDoubleArray(new double[] {});
        poseArray = new Pose2d[tagPosY.length];
        for (int i = 0; i < tagPosY.length; i++)
          poseArray[i] = new Pose2d(tagPosX[i], tagPosY[i], Rotation2d.fromDegrees(0));
      } catch (Exception e) {
        poseArray = new Pose2d[] {defaultPose};
      }
    }

    return poseArray;
  }

  public int[] getTagIds(CAMERA_SERVER location) {
    var tags = tagIds;
    double[] rawTags;
    if (getValidTarget(location)) {
      switch (location) {
        case LIMELIGHT:
          rawTags = m_limelight.getEntry("tid").getDoubleArray(new double[] {});
          tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
          break;
      }
    }
    return tags;
  }

  private void updateVisionPose(CAMERA_SERVER location) {
    if (getValidTarget(location))
      m_swerveDrive
          .getOdometry()
          .addVisionMeasurement(getRobotPose2d(location), getDetectionTimestamp(location));
  }

  private void logData() {
    limelightTargetValid.append(getValidTargetType(CAMERA_SERVER.LIMELIGHT));
  }

  public void initSmartDashboard() {
    SmartDashboard.putData(this);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("pipeline", getPipeline(CAMERA_SERVER.LIMELIGHT));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    // updateVisionPose(CAMERA_SERVER.FUSED_LOCALIZER);
    // searchLimelightPipeline(CAMERA_SERVER.INTAKE);
    updatePipeline();
    // searchforCube(CAMERA_SERVER.INTAKE, 1.0);
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    if (m_limelightLigament2d != null) m_limelightLigament2d.close();
  }
}
