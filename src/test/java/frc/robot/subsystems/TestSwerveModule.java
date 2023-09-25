package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class TestSwerveModule implements AutoCloseable {
  static final double DELTA = 1e-3; // acceptable deviation range
  static final double WAIT_TIME = 0.2;

  SwerveModule m_testModule;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    /* create the TalonFX */
    m_testModule =
        new SwerveModule(
            Constants.SWERVE_DRIVE.SWERVE_MODULE_POSITION.FRONT_LEFT,
            new TalonFX(Constants.CAN.frontLeftTurnMotor),
            new TalonFX(Constants.CAN.frontLeftDriveMotor),
            new CANcoder(Constants.CAN.frontLeftCanCoder),
            0.0);

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    //        Timer.delay(0.100);
    SimHooks.stepTiming(0.1);
  }

  @AfterEach
  void shutdown() throws Exception {
    close();
  }

  @Test
  public void testModuleAngles() {
    var testAngle = 90.0;
    m_testModule.resetAngle(testAngle);

    Timer.delay(WAIT_TIME);

    assertEquals(testAngle, m_testModule.getHeadingRotation2d().getDegrees(), DELTA);
  }

  @Disabled
  public void testModuleSpeed() {
    var testSpeed = 4.0;

    for (int i = 0; i < 50; i++) {
      m_testModule.setDesiredState(new SwerveModuleState(testSpeed, new Rotation2d()), false);
      m_testModule.simulationPeriodic();
      Timer.delay(WAIT_TIME);
    }

    assertEquals(testSpeed, m_testModule.getVelocityMetersPerSecond(), DELTA);
  }

  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    m_testModule.close();
  }
}
