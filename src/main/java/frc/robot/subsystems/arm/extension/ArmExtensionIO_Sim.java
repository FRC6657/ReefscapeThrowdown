package frc.robot.subsystems.arm.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ArmExtensionIO_Sim implements ArmExtensionIO{

    private double voltage = 0;

  private DCMotorSim armExtensionSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), 0.0001, 1),
          DCMotor.getNEO(1));

  public ArmExtensionIO_Sim() {}

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {

    // update sim
    armExtensionSim.update(1 / Constants.CodeConstants.kMainLoopFrequency);

    // inputs
    inputs.volts = voltage; // volts
  }

  @Override
  public void setSpeed(double volts) {
    voltage = MathUtil.clamp(volts, -12, 12);
    armExtensionSim.setInput(voltage);
  }
}
