package frc.robot.subsystems.clawwheels;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class ClawWheelsIO_Real implements ClawWheelsIO{
    
    private TalonSRX clawMotor = new TalonSRX(Constants.CANID.kClawWheels);
    private double setpoint = 0;

    
    public ClawWheelsIO_Real(){
        var motorConfigurator = clawMotor.getConfigurator();
        var motorConfigs = new TalonSRXConfiguration();

        motorConfigs.CurrentLimits = Constants.ClawWheels.currentConfigs;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var kTemp = clawMotor.getDeviceTemp();
        var kCurrent = clawMotor.getSupplyCurrent();

        kTemp.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency / 4);
        kCurrent.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);

        clawMotor.optimizeBusUtilization();

        setVoltage(0);
    }


    @Override
    public void updateInputs(ClawWheelsIOInputs inputs) {
  
      inputs.kTemp = clawMotor.getDeviceTemp().getValueAsDouble();  
      inputs.kCurrent = clawMotor.getSupplyCurrent().getValueAsDouble();
      inputs.kVoltage = clawMotor.getMotorVoltage().getValueAsDouble();
  
      inputs.kSetpoint = setpoint;
      clawMotor.setControl(new VoltageOut(setpoint));
    }


    @Override
    public void setVoltage(double voltage){
        setpoint = voltage;
        clawMotor.setControl(new VoltageOut(setpoint));
    }
}

