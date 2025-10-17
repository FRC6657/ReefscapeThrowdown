package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.CANID;
import frc.robot.Constants.CodeConstants;

public class GyroIO_Real implements GyroIO {

  private final PigeonIMU pigeon = new PigeonIMU(CANID.kPigeon);
  
  private double yaw = pigeon.getYaw();
  private double previousYaw = yaw;
  private double yawVelocity = 0.0; //TODO calculate velocity and update every tick
  

  /** Gyro IO for real robot */
  public GyroIO_Real() {
    pigeon.configFactoryDefault();
    pigeon.setYaw(0);
    //pigeon.getConfigurator().apply(new Pigeon2Configuration()); // Restore Factory Defaults
    //pigeon.getConfigurator().setYaw(0);
    //yaw.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
    //yawVelocity.setUpdateFrequency(CodeConstants.kMainLoopFrequency);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    //inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK); // TODO figure out a way to update whether it is connected
    yaw = pigeon.getYaw();
    inputs.yawPosition = yaw;
    yawVelocity = (yaw - previousYaw)/CodeConstants.kMainLoopFrequency;
    previousYaw = yaw;
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity);
  }
}
