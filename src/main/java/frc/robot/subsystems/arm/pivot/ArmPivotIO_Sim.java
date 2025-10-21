package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;


public class ArmPivotIO_Sim implements ArmPivotIO{
    
    private double voltage = 0;

    private double setpoint = Constants.ArmPivotConstants.kInitialSetpoint;

    private DCMotorSim armPivotSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 0.0001, Constants.ArmPivotConstants.gearRatio),
            DCMotor.getNEO(1)
    );

    public ArmPivotIO_Sim() {}

    @Override
    public void updateInputs(ArmPivotIOInputs inputs){
        armPivotSim.update(1 / Constants.CodeConstants.kMainLoopFrequency);

        inputs.kVelocity = armPivotSim.getAngularVelocityRPM();
        inputs.kTemp = 0;
        inputs.kVoltage = voltage;
        inputs.kCurrent = armPivotSim.getCurrentDrawAmps();
        inputs.kSetpoint = setpoint; 
    }

    @Override
    public void changeSetpoint(double setpoint) {
        voltage = MathUtil.clamp(setpoint, -12, 12);
        armPivotSim.setInput(voltage);
    }

}
