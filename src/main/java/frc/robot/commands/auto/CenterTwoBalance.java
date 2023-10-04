package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

public class CenterTwoBalance extends SequentialCommandGroup {
  public CenterTwoBalance(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist) {

    double maxVel = Units.feetToMeters(8);
    double maxAccel = Units.feetToMeters(8);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(4);
      maxAccel = Units.feetToMeters(4);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    var trajectories = TrajectoryUtils.readTrajectory(pathName, constraints);
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new SetSwerveOdometry(swerveDrive, trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        // new PlotAutoTrajectory(fieldSim, pathName, trajectories),

            
           //Rev Flywheel
        
        new WaitCommand(1),
            //run kicker
        new ParallelCommandGroup(
            swerveCommands.get(0),
            new SequentialCommandGroup(
                // new WaitCommand(0.75),
                //stow
            )
            
        ),
        new SequentialCommandGroup(
            // new WaitCommand(2),
            //intakeDown&&intake
        ),

        // new ParallelCommandGroup(
        //     swerveCommands.get(1),
        //     //stow,
        // ),

    
        //run flywheel
        
        new AutoBalance(swerveDrive).withTimeout(4)
            .andThen(() -> swerveDrive.drive(0, 0.0000001, 0, false, false)),

        //run kicker
        new AutoBalance(swerveDrive),

        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
