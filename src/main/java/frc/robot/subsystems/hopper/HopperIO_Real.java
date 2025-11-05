package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class HopperIO_Real implements HopperIO {

  private TalonSRX hopperMotor = new TalonSRX(Constants.CANID.kClawWheels);
  private double setpoint = 0;

  public HopperIO_Real() {
    hopperMotor.setNeutralMode(NeutralMode.Coast);
    setSpeed(0);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.current = hopperMotor.getSupplyCurrent();
    inputs.voltage = hopperMotor.getMotorOutputVoltage();
    inputs.setpoint = setpoint;
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
    hopperMotor.set(TalonSRXControlMode.PercentOutput, setpoint);
  }
}
