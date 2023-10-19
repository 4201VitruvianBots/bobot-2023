package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

public class DriveStraight extends SequentialCommandGroup {

  public DriveStraight(SwerveDrive swerveDrive, FieldSim fieldSim) {

    double maxVel = Units.feetToMeters(16);
    double maxAccel = Units.feetToMeters(16);

    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(8);
      maxAccel = Units.feetToMeters(8);
    }

    var trajectory =
        PathPlanner.generatePath(
            new PathConstraints(maxVel, maxAccel),
            new PathPoint(
                new Translation2d(3, 3), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
            new PathPoint(
                new Translation2d(8, 3), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));

    var trajectories = new ArrayList<PathPlannerTrajectory>();
    trajectories.add(trajectory);

    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        new SetSwerveOdometry(swerveDrive, new Pose2d(3, 3, Rotation2d.fromDegrees(0)), fieldSim),
        new PlotAutoTrajectory(fieldSim, "DriveStraight", trajectories),
        new WaitCommand(1),
        swerveCommands.get(0).andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
