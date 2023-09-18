package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Wrist extends SubsystemBase {
    private double m_wristDesiredSetpointRadians;
    
    private final TalonFX wrist = new TalonFX(CAN.wristMotor);

    public void setDesiredSetpoint(Constants.SETPOINT desiredSetpoint) {
        m_wristDesiredSetpointRadians = desiredSetpoint.getWristSetpointRadians();
      }
      public double getWristSetpointRadians() {
        return wristSetpointRadians;
      }
}
