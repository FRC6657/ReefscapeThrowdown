package frc.robot.subsystems.armpivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.armpivot.ArmPivotIO.ArmPivotIOInputs;

public class ArmPivotIO_Sim {
    
    private double voltage = 0;

    private double setpoint = Constants.ArmPivotConstants.kSetpoint;

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
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12, 12);
        armPivotSim.setInput(voltage);
    }

}
