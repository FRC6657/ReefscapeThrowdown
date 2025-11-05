package frc.robot.subsystems.arm.extension;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.*;

public class ArmExtensionIO_Real implements ArmExtensionIO {
  private TalonSRX motor;

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    motor = new TalonSRX(CANID.kClawExtension);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 5, 0));
  }

  @Override
  public void setSpeed(double dutycyc) {
    motor.set(TalonSRXControlMode.PercentOutput, dutycyc);
  }
}
