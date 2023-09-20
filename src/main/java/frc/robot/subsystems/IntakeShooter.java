package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class IntakeShooter extends SubsystemBase {

  private final TalonFX intake = new TalonFX(CAN.intakeMotor);
  private final TalonFX flywheel = new TalonFX(CAN.flywheelMotor);

  public void setIntakePercentOutput(double percentOutput) {
    intake.setControl(new DutyCycleOut(percentOutput));
  }

  public void setFlywheelPercentOutput(double percentOutput) {
    flywheel.setControl(new DutyCycleOut(percentOutput));
  }
}
