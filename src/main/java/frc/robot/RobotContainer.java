// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.autopaths.BumpThree;
import frc.robot.commands.auto.autopaths.CenterTwoBalance;
import frc.robot.commands.auto.autopaths.DriveForward;
import frc.robot.commands.flywheel.RunFlywheel;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.kicker.RunKickerIn;
import frc.robot.commands.kicker.RunKickerOut;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.wrist.RunWristJoystick;
import frc.robot.commands.wrist.SetWristManual;
import frc.robot.commands.wrist.WristHandler;
import frc.robot.commands.wrist.ZeroWristEncoder;
import frc.robot.constants.BASE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.INTAKE;
import frc.robot.constants.USB;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Wrist m_wrist = new Wrist();
  private final IntakeShooter m_intakeShooter = new IntakeShooter();
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);

  private final Trigger[] leftJoystickTriggers = new Trigger[2]; // left joystick buttons
  private final Trigger[] rightJoystickTriggers = new Trigger[2]; // right joystick buttons

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    // Configure the trigger bindings
    configureBindings();

    initializeAutoChooser();

    initializeButtonCommands();
  }

  public void initializeButtonCommands() {
    SmartDashboard.putData(new ZeroWristEncoder(m_wrist));
  }

  public void initializeSubsystems() {
    if (RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive(
              m_swerveDrive,
              () -> leftJoystick.getRawAxis(1),
              () -> leftJoystick.getRawAxis(0),
              () -> rightJoystick.getRawAxis(0)));
    } else {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive(
              m_swerveDrive,
              () -> -leftJoystick.getRawAxis(1),
              () -> -leftJoystick.getRawAxis(0),
              () -> -leftJoystick.getRawAxis(2)));
    }
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    for (int i = 0; i < leftJoystickTriggers.length; i++)
      leftJoystickTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightJoystickTriggers.length; i++)
      rightJoystickTriggers[i] = new JoystickButton(rightJoystick, (i + 1));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    xboxController
        .leftTrigger()
        .whileTrue(
            new RunIntake(m_intakeShooter)
                .alongWith(new WristHandler(m_wrist, SETPOINT.INTAKING_LOW_CUBE, xboxController::getLeftY)));

    xboxController.x().whileTrue(new RunKickerOut(m_intakeShooter));

    xboxController.rightTrigger().whileTrue(new RunKickerIn(m_intakeShooter));

    xboxController
        .y()
        .whileTrue(
            new RunFlywheel(m_intakeShooter, INTAKE.FLYWHEEL_SPEED.HIGH)
                .alongWith(new WristHandler(m_wrist, SETPOINT.SCORE_HIGH_CUBE, xboxController::getLeftY)));

    xboxController
        .b()
        .whileTrue(
            new RunFlywheel(m_intakeShooter, INTAKE.FLYWHEEL_SPEED.MEDIUM)
                .alongWith(new WristHandler(m_wrist, SETPOINT.SCORE_MID_CUBE, xboxController::getLeftY)));

    xboxController
        .a()
        .whileTrue(
            new RunFlywheel(m_intakeShooter, INTAKE.FLYWHEEL_SPEED.LOW)
                .alongWith(new WristHandler(m_wrist, SETPOINT.SCORE_LOW_REVERSE, xboxController::getLeftY)));

    xboxController.povDown().whileTrue(new WristHandler(m_wrist, SETPOINT.STOWED, xboxController::getLeftY));

    xboxController
        .rightStick()
        .whileTrue((new WristHandler(m_wrist, BASE.SETPOINT.INTAKING_LOW_CUBE, xboxController::getLeftY)));
  }

  public void disableInit() {
    m_swerveDrive.setNeutral(NeutralMode.Coast);
    m_wrist.setNeutralMode(IdleMode.kCoast);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutral(NeutralMode.Brake);
    m_wrist.setNeutralMode(IdleMode.kBrake);
  }

  public void autonomousInit() {
    m_swerveDrive.setNeutral(NeutralMode.Brake);
    m_wrist.setNeutralMode(IdleMode.kBrake);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  public void initializeAutoChooser() {
    m_autoChooser.addOption(
        "CenterTwoBalance",
        new CenterTwoBalance(
            "CenterTwoBalance", m_swerveDrive, m_fieldSim, m_wrist, m_intakeShooter));

    m_autoChooser.addOption(
        "BumpThree",
        new BumpThree("BumpThree", m_swerveDrive, m_fieldSim, m_wrist, m_intakeShooter));

    m_autoChooser.addOption(
        "DriveForward",
        new DriveForward("DriveForward", m_swerveDrive, m_fieldSim));

    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public void periodic() {
    m_fieldSim.periodic();

    // Absolute definition of jank right here. Please change this before Beach Blitz
    // :nate:
    // TODO: Fix
    if (!(xboxController.leftTrigger().getAsBoolean() || xboxController.y().getAsBoolean() || xboxController.b().getAsBoolean() || xboxController.a().getAsBoolean() || xboxController.povDown().getAsBoolean()) && xboxController.getLeftY() > 0.1)  {
      WristHandler wristHandler = new WristHandler(m_wrist, null, xboxController::getLeftY);
      wristHandler.schedule();
    }
  }
}
