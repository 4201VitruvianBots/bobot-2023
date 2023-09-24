package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;

public class IntakeShooter extends SubsystemBase {

  private final TalonFX intake = new TalonFX(CAN.intakeMotor);
  private final TalonFX flywheel = new TalonFX(CAN.flywheelMotor);

  public void setIntakePercentOutput(double percentOutput) {
    intake.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setFlywheelPercentOutput(double percentOutput) {
    flywheel.set(ControlMode.PercentOutput, percentOutput);
  }
}
