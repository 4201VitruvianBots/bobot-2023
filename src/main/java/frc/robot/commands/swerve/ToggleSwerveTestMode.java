// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleSwerveTestMode extends ParallelCommandGroup {
  /** Creates a new ToggleSwerveTestMode. */
  private final SwerveDrive m_swerveDrive;

  public ToggleSwerveTestMode(SwerveDrive swerveDriveSubsystem) {
    m_swerveDrive = swerveDriveSubsystem;
    addRequirements(m_swerveDrive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SwerveSetTestMode(swerveDriveSubsystem),
        new SetSwerveNeutralMode(swerveDriveSubsystem, NeutralMode.Brake)
            .andThen(() -> swerveDriveSubsystem.drive(0, 0, 0, isFinished(), isFinished())));
  }
}
