package frc.robot.subsystems.clawwheels;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClawWheelsIO_Sim implements ClawWheelsIO{
    private double clawWheelsVoltage = 0;

    private DCMotorSim clawWheelSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, 1), DCMotor.getNEO(1));



@Override
  public void updateInputs(ClawWheelsIOInputs inputs) {
    
    inputs.kCurrent = clawWheelSim.getCurrentDrawAmps(); 
    
  }

}