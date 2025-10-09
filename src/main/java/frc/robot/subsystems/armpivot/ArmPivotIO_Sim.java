package frc.robot.subsystems.armpivot;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

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

        inputs.position = armPivotSim.getAngularPositionRotations();
        inputs.velocity = armPivotSim.getAngularVelocityRPM();
        inputs.temp = 0;
        inputs.voltage = voltage;
        inputs.current = armPivotSim.getCurrentDrawAmps();
        inputs.setpoint = setpoint; 
    }

    @Override
    public void setVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12, 12);
        armPivotSim.setInput(voltage);
    }

}
