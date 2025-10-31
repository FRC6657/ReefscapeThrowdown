package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANID;
import frc.robot.Constants.CodeConstants;

public class GyroIO_Real implements GyroIO {

  private final PigeonIMU pigeon = new PigeonIMU(CANID.kPigeon);

  private double yaw = pigeon.getYaw();
  private double previousYaw = yaw;
  private double yawVelocity = 0.0; // TODO calculate velocity and update every tick

  /** Gyro IO for real robot */
  public GyroIO_Real() {
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    yaw = pigeon.getYaw();
    inputs.yawPosition = yaw;
    yawVelocity = (yaw - previousYaw) / CodeConstants.kMainLoopFrequency;
    previousYaw = yaw;
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity);
  }
}
