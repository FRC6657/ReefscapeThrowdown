package frc.robot.subsystems.clawwheels;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClawWheelsIO_Sim implements ClawWheelsIO{
    private double voltage = 0;
    double setpoint = 0.0;
    double speed = 0.0;

    private DCMotorSim clawWheelSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, 1), DCMotor.getNEO(1));


@Override
  public void updateInputs(ClawWheelsIOInputs inputs) {
    inputs.kTemp = 0;
    inputs.kCurrent = clawWheelSim.getCurrentDrawAmps(); 
    inputs.kVoltage = voltage;
    inputs.kSetpoint = setpoint;
  }
 
  
  @Override
  public void setSpeed(double setpoint){
     setpoint = speed;
  }
}