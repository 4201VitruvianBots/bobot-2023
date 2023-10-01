package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;

public class IntakeShooter extends SubsystemBase {

  private final CANSparkMax kickerMotor = new CANSparkMax(CAN.kickerMotor, MotorType.kBrushless);
  private final CANSparkMax flywheelMotor = new CANSparkMax(CAN.flywheelMotor, MotorType.kBrushless);

  private final CANSparkMax[] motors = {kickerMotor, flywheelMotor};

  public IntakeShooter() {
    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
    }
  }
  public void setKickerPercentOutput(double percentOutput) {
    kickerMotor.set(percentOutput);
  }

  public void setFlywheelPercentOutput(double percentOutput) {
    flywheelMotor.set(percentOutput);
  }
}
