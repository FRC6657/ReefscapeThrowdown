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

        inputs.kvelocity = armPivotSim.getAngularVelocityRPM();
        inputs.ktemp = 0;
        inputs.kvoltage = voltage;
        inputs.kcurrent = armPivotSim.getCurrentDrawAmps();
        inputs.setpoint = setpoint; 
    }

    @Override
    public void changeSetpoint(double setpoint) {
        voltage = MathUtil.clamp(setpoint, -12, 12);
        armPivotSim.setInput(voltage);
    }

}
