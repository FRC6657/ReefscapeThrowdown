package frc.robot.subsystems.arm.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawWheelsIO {

  @AutoLog
  public static class ClawWheelsIOInputs {
    public double kSetpoint = 0.0;
    public double kVoltage = 0.0;
    public double kCurrent = 0.0;
  }

  public default void updateInputs(ClawWheelsIOInputs inputs) {}

  /**
   * Sets the speed of the claw wheels.
   *
   * @param speed The speed setpoint for the claw wheels. (-1,1)
   */
  public default void setSpeed(double speed) {}
}
