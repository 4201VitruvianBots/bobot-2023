package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.INTAKE;

public class IntakeShooter extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(CAN.intakeMotor);
  private final DutyCycleOut intakeDutyControl = new DutyCycleOut(0);
  private final TalonFX flywheelMotor = new TalonFX(CAN.flywheelMotor);
  private final DutyCycleOut flywheelDutyControl = new DutyCycleOut(0);

  private final TalonFX[] motors = {intakeMotor, flywheelMotor};

  public IntakeShooter() {
    for (TalonFX motor : motors) {
      // factory default configs
      motor.setInverted(true);

      // Feedback sensor configuration
      // TODO: might not be needed
      FeedbackConfigs feedbackConfig = new FeedbackConfigs();
      feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      motor.getConfigurator().apply(new FeedbackConfigs());

      // PID configuration
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kV = INTAKE.kV;
      slot0Configs.kP = INTAKE.kP;
      motor.getConfigurator().apply(slot0Configs);
      
      // set current limit on TalonFX motors
      // motor.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 0.1));
      // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 0.1));

      // Sets status frame period to 2 ms
      intakeDutyControl.UpdateFreqHz = 500;
      intakeDutyControl.configTimeout = 0.255;

      // neutral mode config not supported in Phoenix 6

      // class member variable
      VoltageOut m_request = new VoltageOut(0);
      // command 10 V output
      motor.setControl(m_request.withOutput(10.0));
    }
  }
  public void setIntakePercentOutput(double percentOutput) {
    intakeMotor.setControl(intakeDutyControl.withOutput(percentOutput));
  }

  public void setFlywheelPercentOutput(double percentOutput) {
    flywheelMotor.setControl(flywheelDutyControl.withOutput(percentOutput));
  }
}
