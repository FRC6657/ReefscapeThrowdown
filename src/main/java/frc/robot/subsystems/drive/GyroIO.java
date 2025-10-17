package frc.robot.subsystems.drive;

// import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Generic Gyro IO */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPosition = 0.0;
    public double yawVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
