package frc.robot.subsystems.clawwheels;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClawWheelsIO_Real implements ClawWheelsIO{
    
    private final TalonFX clawMotor;
    private double setpoint = 0;


    public ClawWheelsIO_Real(int canID){
        clawMotor = new TalonFX(canID);
    }
    
    public void SetVoltage(){
        
    }


}

