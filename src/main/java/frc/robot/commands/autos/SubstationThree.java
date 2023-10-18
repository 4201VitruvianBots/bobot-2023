package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.robot.utils.TrajectoryUtils;

public class SubstationThree extends SequentialCommandGroup {
  public SubstationThree(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim) {

    double maxVel = Units.feetToMeters(16);
    double maxAccel = Units.feetToMeters(16);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(4);
      maxAccel = Units.feetToMeters(4);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    var m_trajectories = TrajectoryUtils.readTrajectory(pathName, constraints);
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),

        /** Runs Path with Intaking cube during */
        swerveCommands.get(0),
        new WaitCommand(0.2),
        swerveCommands.get(1),
        new WaitCommand(0.2),
        swerveCommands.get(2)
                .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}