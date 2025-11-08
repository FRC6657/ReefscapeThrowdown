package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants.CANID;

public class GyroIO_Real implements GyroIO {

  private final PigeonIMU pigeon = new PigeonIMU(CANID.kPigeon);

  /** Gyro IO for real robot */
  public GyroIO_Real() {
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = pigeon.getFusedHeading();
  }

  @Override
  public void zeroYaw() {
    pigeon.setYaw(0);
  }
}
