package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIO_Sim implements HopperIO {
  private double voltage = 0;
  double setpoint = 0.0;
  double speed = 0.0;

  private DCMotorSim clawWheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, 1), DCMotor.getNEO(1));

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.current = clawWheelSim.getCurrentDrawAmps();
    inputs.voltage = voltage;
    inputs.setpoint = setpoint;
  }

  @Override
  public void setSpeed(double setpoint) {
    setpoint = speed;
  }
}
