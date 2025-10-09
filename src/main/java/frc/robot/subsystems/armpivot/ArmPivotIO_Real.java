package frc.robot.subsystems.armpivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmPivotIO_Real implements ArmPivotIO {

    // Pivot Motor Controller
    TalonFX pivotMotor = new TalonFX(Constants.CANID.kPivot);

    // Remember unit circle!!
    private double kSetpoint = Constants.ArmPivotConstants.kSetpoint;

   public ArmPivotIO_Real() {

    var motorConfigurator = pivotMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio = 
       1.0 / Constants.ArmPivotConstants.gearRatio;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    motorConfigurator.apply(motorConfigs);

    var kTemp = pivotMotor.getDeviceTemp();
    var kVoltage = pivotMotor.getMotorVoltage();
    var kCurrent = pivotMotor.getSupplyCurrent();

    kTemp.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency /4);
    kVoltage.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);
    kCurrent.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);
    
    pivotMotor.optimizeBusUtilization();

    changeSetpoint(0);

   }

   @Override
   public void updateInputs(ArmPivotIOInputs inputs) {

    inputs.kTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.kCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.kVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

    inputs.kSetpoint = kSetpoint;

    pivotMotor.setControl(new VoltageOut(kSetpoint * 12));
   }

   @Override
   public void changeSetpoint(double setpoint) {
    kSetpoint = setpoint;
    pivotMotor.setControl(new VoltageOut(kSetpoint * 12));
   }
    
}
