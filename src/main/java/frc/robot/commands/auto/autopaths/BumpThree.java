package frc.robot.commands.auto.autopaths;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.PlotAutoTrajectory;
import frc.robot.commands.auto.autocommands.AutoRunIntakeMotors;
import frc.robot.commands.auto.autocommands.AutoWristSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.INTAKE.FLYWHEEL_SPEED;
import frc.robot.constants.INTAKE.KICKER_SPEED;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

public class BumpThree extends SequentialCommandGroup {
  public BumpThree(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      IntakeShooter intakeShooter) {

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
        new PlotAutoTrajectory(fieldSim, pathName, trajectories),
        new ParallelCommandGroup(
            new AutoRunIntakeMotors(intakeShooter, FLYWHEEL_SPEED.HIGH, KICKER_SPEED.NONE),
            new AutoWristSetpoint(wrist, SETPOINT.SCORE_HIGH_CUBE)).withTimeout(1),
        new WaitCommand(1),
        new AutoRunIntakeMotors(intakeShooter, FLYWHEEL_SPEED.HIGH, KICKER_SPEED.SHOOT).withTimeout(0.5),
        new ParallelCommandGroup(
            new AutoRunIntakeMotors(intakeShooter, FLYWHEEL_SPEED.NONE, KICKER_SPEED.NONE),
            new AutoWristSetpoint(wrist, SETPOINT.STOWED)).withTimeout(0.25),
                swerveCommands.get(0).withTimeout(trajectories.get(0).getTotalTimeSeconds()),
        //     new ParallelCommandGroup(
        //         swerveCommands.get(1),
        //         new AutoWristSetpoint(wrist, SETPOINT.STOWED),
        //         new AutoRunIntakeMotors(intakeShooter, FLYWHEEL_SPEED.HIGH, KICKER_SPEED.STALL)).withTimeout(trajectories.get(1).getTotalTimeSeconds()),

        // // run flywheel
        // new AutoWristSetpoint(wrist, SETPOINT.SCORE_HIGH_CUBE),
        // new AutoRunIntakeMotors(intakeShooter, FLYWHEEL_SPEED.MEDIUM, KICKER_SPEED.SHOOT),
        // new WaitCommand(0.25),
        // new ParallelCommandGroup(
        //     swerveCommands.get(2),
        //     new AutoWristSetpoint(wrist, SETPOINT.STOWED),
        //     new AutoRunIntakeMotors(intakeShooter, FLYWHEEL_SPEED.NONE, KICKER_SPEED.NONE)),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
