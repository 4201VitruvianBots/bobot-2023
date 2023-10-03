package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public final class INTAKE {
  public static final DCMotor gearBox = DCMotor.getNEO(2);
  public static double kP = 0.2;

  public static final double maxRPM = 5700;

  public static enum FLYWHEEL_SPEED {
    LOW(-0.3),
    MEDIUM(-0.5),
    HIGH(-0.7);

    public final double speed;

    private FLYWHEEL_SPEED(double speed) {
      this.speed = speed;
    }

    public double get() {
      return this.speed;
    }
  }
}
