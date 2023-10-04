package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public final class INTAKE {
  public static final DCMotor gearBox = DCMotor.getNEO(2);
  public static double kP = 0.2;

  public static final double maxRPM = 5700;

  public static enum FLYWHEEL_SPEED {
    INTAKE(0.25),
    NONE(0.0),
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

  public static enum KICKER_SPEED {
    INTAKE(-0.85),
    STALL(-0.1),
    NONE(0.0),
    SHOOT(0.5);

    public final double speed;

    private KICKER_SPEED(double speed) {
      this.speed = speed;
    }

    public double get() {
      return this.speed;
    }
  }
  }

  public static final double kWristMaxAccel = 0.0;
  public static final double kWristMaxVel = 0.0;

  public static final double kWristP = 0.0;
  public static final double kWristI = 0.0;
  public static final double kWristD = 0.0;

  public static final double kWristRadiansToEncoderUnits = 0.0;
  
}
