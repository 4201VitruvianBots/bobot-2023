package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;

public class TestCtreUtils implements AutoCloseable {
  static final double DELTA = 1e-3; // acceptable deviation range
  static final double WAIT_TIME = 0.2;

  TalonFX m_testMotor;

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    /* create the TalonFX */
    m_testMotor = new TalonFX(0);

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    /* delay ~100ms so the devices can start up and enable */
    //        Timer.delay(0.100);
    SimHooks.stepTiming(0.1);
  }

  @AfterEach
  void shutdown() {
    close();
  }

  @Disabled
  public void testSensorRatioConfig() {
    var testMotorSim = m_testMotor.getSimState();

    var testConfig = CtreUtils.generateTurnMotorConfig();
    testConfig.Feedback.SensorToMechanismRatio = Constants.SWERVE_MODULE.kTurningMotorGearRatio;
    m_testMotor.getConfigurator().apply(testConfig);

    testMotorSim.setRawRotorPosition(-1);
    m_testMotor.getPosition().waitForUpdate(WAIT_TIME);

    var expectedRotation = 1.0 / Constants.SWERVE_MODULE.kTurningMotorGearRatio;
    var testPosition = m_testMotor.getPosition().getValue();

    assertEquals(expectedRotation, testPosition, DELTA);

    testMotorSim.setRawRotorPosition(1);
    m_testMotor.getPosition().waitForUpdate(WAIT_TIME);

    expectedRotation = -1.0 / Constants.SWERVE_MODULE.kTurningMotorGearRatio;
    testPosition = m_testMotor.getPosition().getValue();

    assertEquals(expectedRotation, testPosition, DELTA);
  }

  @Override
  public void close() {
    /* destroy our TalonFX object */
    m_testMotor.close();
  }
}
