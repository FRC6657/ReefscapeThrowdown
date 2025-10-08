package frc.robot.subsystems.clawwheels;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import frc.robot.Constants;

public class ClawWheelsIO_Real implements ClawWheelsIO{
    
    private TalonSRX clawMotor = new TalonSRX(Constants.CANID.kClawWheels);
    private double setpoint = 0;

    
    public ClawWheelsIO_Real(){

       
        clawMotor.setNeutralMode(NeutralMode.Brake);

        setSpeed(0);
    }


    @Override
    public void updateInputs(ClawWheelsIOInputs inputs) {
    
      inputs.kCurrent = clawMotor.getSupplyCurrent();
      inputs.kVoltage = clawMotor.getMotorOutputVoltage();
    
      inputs.kSetpoint = setpoint;
      clawMotor.set(TalonSRXControlMode.PercentOutput, setpoint); // we can probably get rid of periodic updates, but I doubt it would fill the CAN Bus usage
    }


    @Override
    public void setSpeed(double speed){
        setpoint = speed;
        clawMotor.set(TalonSRXControlMode.PercentOutput, setpoint);
    }
}

