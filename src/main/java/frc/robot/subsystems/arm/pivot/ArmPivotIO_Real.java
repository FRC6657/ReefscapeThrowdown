package frc.robot.subsystems.arm.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ArmPivotIO_Real implements ArmPivotIO {

    // Pivot Motor Controller
    TalonFX pivotMotor = new TalonFX(Constants.CANID.kPivot);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0); //TODO: fix this default setpoint

    // Remember unit circle!!
    private double kSetpoint = Constants.ArmPivotConstants.kInitialSetpoint; //TODO: k is usually a prefix for constants. Change name.

   public ArmPivotIO_Real() {

    var motorConfigurator = pivotMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio = //TODO: Remove 1/. Reductions should be > 1
       1.0 / Constants.ArmPivotConstants.gearRatio;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    motorConfigs.Slot0 = Constants.ArmPivotConstants.motorSlot0;
    motorConfigs.CurrentLimits = Constants.ArmPivotConstants.currentConfigs;
    motorConfigs.MotionMagic = Constants.ArmPivotConstants.kMotionMagicConfig;
    motorConfigurator.apply(motorConfigs);

    //TODO: set position update frequency

    var kTemp = pivotMotor.getDeviceTemp();
    var kVoltage = pivotMotor.getMotorVoltage();
    var kCurrent = pivotMotor.getSupplyCurrent();

    kTemp.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency /4);
    kVoltage.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);
    kCurrent.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);
   
    pivotMotor.optimizeBusUtilization();

    changeSetpoint(Constants.ArmPivotConstants.kInitialSetpoint); //TODO: This is is the same as the home setpoint.
   }

   @Override
   public void updateInputs(ArmPivotIOInputs inputs) {

    inputs.kTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.kCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.kVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.kPosition = pivotMotor.getPosition().getValueAsDouble(); //TODO: Log degrees instead of mechanism rotations
    inputs.kSetpoint = kSetpoint;
    //TODO: Log velocity and acceleration in deg/s and deg/s/s

    pivotMotor.setControl(motionMagicVoltage.withPosition(kSetpoint)); //TODO convert setpoint back to mechanism rotations
   }

   @Override
   public void changeSetpoint(double setpoint) {
    kSetpoint = setpoint; //TODO: needs clamping
   }
    
}
