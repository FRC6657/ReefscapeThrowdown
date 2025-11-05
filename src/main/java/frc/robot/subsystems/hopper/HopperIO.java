package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public double setpoint = 0.0;
    public double voltage = 0.0;
    public double current = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  /**
   * Sets the speed of the hopper.
   *
   * @param speed The speed setpoint for the hopper. (-1,1)
   */
  public default void setSpeed(double speed) {}
}
