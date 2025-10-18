package frc.robot.subsystems.arm.claw;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class ClawWheelsIO_Real implements ClawWheelsIO {

    private TalonSRX clawMotor = new TalonSRX(Constants.CANID.kClawWheels);
    private double setpoint = 0;

    public ClawWheelsIO_Real() {
        clawMotor.setNeutralMode(NeutralMode.Brake);
        setSpeed(0);
    }

    @Override
    public void updateInputs(ClawWheelsIOInputs inputs) {
        inputs.kCurrent = clawMotor.getSupplyCurrent(); //TODO: Probably dont need to log this
        inputs.kVoltage = clawMotor.getMotorOutputVoltage();
        inputs.kSetpoint = setpoint;
    }

    @Override
    public void setSpeed(double speed) {
        setpoint = speed;
        clawMotor.set(TalonSRXControlMode.PercentOutput, setpoint); //TODO: This should be put in update inputs.
    }
}
