package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class HopperIO_Real implements HopperIO {

  private TalonSRX hopperMotor = new TalonSRX(Constants.CANID.kHopper);
  private double setpoint = 0;

  public HopperIO_Real() {
    hopperMotor.setNeutralMode(NeutralMode.Coast);
    hopperMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0));
    hopperMotor.setInverted(false);
    setSpeed(0);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.current = hopperMotor.getSupplyCurrent();
    inputs.voltage = hopperMotor.getMotorOutputVoltage();
    inputs.setpoint = setpoint;
    hopperMotor.set(TalonSRXControlMode.PercentOutput, setpoint);
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
  }
}
