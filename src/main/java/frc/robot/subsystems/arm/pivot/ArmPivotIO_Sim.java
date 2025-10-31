package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ArmPivotIO_Sim implements ArmPivotIO {

  private double voltage = 0;

  private double setpoint = Constants.ArmPivotConstants.initialSetpoint;

  private DCMotorSim armPivotSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getFalcon500(1), 0.0001, Constants.ArmPivotConstants.gearRatio),
          DCMotor.getFalcon500(1));

  private PIDController armPID = new PIDController(1, 0, 0);

  public ArmPivotIO_Sim() {
    armPivotSim.setAngle(Units.degreesToRadians(-90));
  }

  @Override
  public void updateInputs(ArmPivotIOInputs inputs) {

    armPivotSim.setInputVoltage(
        armPID.calculate(armPivotSim.getAngularPositionRotations() * 360, setpoint));
    armPivotSim.update(1 / Constants.CodeConstants.kMainLoopFrequency);

    inputs.kVelocity = armPivotSim.getAngularVelocityRPM();
    inputs.kPosition = armPivotSim.getAngularPositionRotations() * 360;
    inputs.kTemp = 0;
    inputs.kVoltage = voltage;
    inputs.kCurrent = armPivotSim.getCurrentDrawAmps();
    inputs.kSetpoint = setpoint;
  }

  @Override
  public void changeSetpoint(double setpoint) {
    this.setpoint = MathUtil.clamp(setpoint, -90, 90);
  }
}
