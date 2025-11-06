package frc.robot.subsystems.arm.claw;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class ClawWheelsIO_Real implements ClawWheelsIO {

  private TalonSRX clawMotor;
  private double setpoint = 0;

  public ClawWheelsIO_Real() {
    clawMotor = new TalonSRX(Constants.CANID.kClawWheels);
    clawMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0));
    clawMotor.setNeutralMode(NeutralMode.Brake);
    clawMotor.setInverted(false);
  }

  @Override
  public void updateInputs(ClawWheelsIOInputs inputs) {
    inputs.kCurrent = clawMotor.getSupplyCurrent();
    inputs.kVoltage = clawMotor.getMotorOutputVoltage();
    inputs.kSetpoint = setpoint;
    clawMotor.set(TalonSRXControlMode.PercentOutput, setpoint);
  }

  @Override
  public void setSpeed(double speed) {
    setpoint = speed;
  }
}
