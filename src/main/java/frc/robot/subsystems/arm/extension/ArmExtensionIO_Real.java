package frc.robot.subsystems.arm.extension;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.*;

public class ArmExtensionIO_Real implements ArmExtensionIO {
  private TalonSRX motor;
  private double setpoint = 0;

  public ArmExtensionIO_Real() {
    motor = new TalonSRX(CANID.kClawExtension);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 5, 0));
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    motor.set(TalonSRXControlMode.PercentOutput, setpoint);
    inputs.volts = motor.getMotorOutputVoltage();
  }

  @Override
  public void setSpeed(double dutycyc) {
    setpoint = dutycyc;
  }
}
