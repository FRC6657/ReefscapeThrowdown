package frc.robot.subsystems.arm.extension;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.ArmConstants;

public class ArmExtensionIO_Real implements ArmExtensionIO {
  private TalonSRX motor;

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    motor = new TalonSRX(ArmConstants.kArmExtend);
  }

  @Override
  public void setSpeed(double dutycyc) {
    motor.set(TalonSRXControlMode.PercentOutput, dutycyc);
  }
}
