package frc.robot.subsystems.arm.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class ArmPivotIO_Real implements ArmPivotIO {

  // Pivot Motor Controller
  TalonFX pivotMotor = new TalonFX(Constants.CANID.kPivot);
  private MotionMagicVoltage motionMagicVoltage =
      new MotionMagicVoltage(Constants.ArmPivotConstants.initialSetpoint / 360);

  private double kSetpoint = Constants.ArmPivotConstants.initialSetpoint;

  public ArmPivotIO_Real() {

    var motorConfigurator = pivotMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio = Constants.ArmPivotConstants.gearRatio;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Slot0 = Constants.ArmPivotConstants.motorSlot0;
    motorConfigs.CurrentLimits = Constants.ArmPivotConstants.currentConfigs;
    motorConfigs.MotionMagic = Constants.ArmPivotConstants.kMotionMagicConfig;
    motorConfigurator.apply(motorConfigs);

    var kPosition = pivotMotor.getPosition();
    var kTemp = pivotMotor.getDeviceTemp();
    var kVoltage = pivotMotor.getMotorVoltage();
    var kCurrent = pivotMotor.getSupplyCurrent();

    kTemp.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency / 4);
    kVoltage.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);
    kCurrent.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);
    kPosition.setUpdateFrequency(Constants.CodeConstants.kMainLoopFrequency);

    pivotMotor.optimizeBusUtilization();

    pivotMotor.setPosition(Constants.ArmPivotConstants.initialSetpoint / 360);

    changeSetpoint(Constants.ArmPivotConstants.initialSetpoint);
  }

  @Override
  public void updateInputs(ArmPivotIOInputs inputs) {

    inputs.kTemp = pivotMotor.getDeviceTemp().getValueAsDouble();
    inputs.kCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.kVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.kPosition = pivotMotor.getPosition().getValueAsDouble() * 360;
    inputs.kVelocity = pivotMotor.getVelocity().getValueAsDouble() * 360;
    inputs.kAcceleration = pivotMotor.getAcceleration().getValueAsDouble() * 360;
    inputs.kSetpoint = kSetpoint;

    pivotMotor.setControl(motionMagicVoltage.withPosition(kSetpoint / 360));
  }

  @Override
  public void changeSetpoint(double setpoint) {
    kSetpoint =
        MathUtil.clamp(
            setpoint,
            Constants.ArmPivotConstants.initialSetpoint,
            Constants.ArmPivotConstants.maxStepoint);
  }
}
