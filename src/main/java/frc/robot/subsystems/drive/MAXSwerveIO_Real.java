package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants.SwerveModuleInformation;
import frc.robot.Constants.MAXSwerveConstants;
import org.littletonrobotics.junction.Logger;

public class MAXSwerveIO_Real implements MAXSwerveIO {

  private SparkMax driveMotor;
  private SparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private AbsoluteEncoder turnEncoder;

  // Onboard PID controller
  private SparkClosedLoopController drivePID;
  private SparkClosedLoopController turnPID;

  private SparkMaxConfig driveConfig = new SparkMaxConfig();
  private SparkMaxConfig turnConfig = new SparkMaxConfig();

  SwerveModuleInformation moduleInformation;

  double turnSetpoint = 0.0;
  double driveSetpoint = 0.0;

  double arbFF = 0.0;

  double lastDriveSetpoint = 0.0;

  public MAXSwerveIO_Real(SwerveModuleInformation moduleInformation) {

    this.moduleInformation = moduleInformation;

    driveMotor = new SparkMax(moduleInformation.driveCAN_ID, MotorType.kBrushless);
    turnMotor = new SparkMax(moduleInformation.turnCAN_ID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // relative encoder
    driveConfig.closedLoop.pid(
        MAXSwerveConstants.kDriveP, MAXSwerveConstants.kDriveI, MAXSwerveConstants.kDriveD);
    driveConfig.closedLoop.outputRange(
        MAXSwerveConstants.kDriveMinOutput, MAXSwerveConstants.kDriveMaxOutput);
    driveConfig.closedLoop.velocityFF(1d / MAXSwerveConstants.kMaxDriveSpeed);

    driveConfig.encoder.positionConversionFactor(
        MAXSwerveConstants.kDriveEncoderPositionFactor); // meters
    driveConfig.encoder.velocityConversionFactor(
        MAXSwerveConstants.kDriveEncoderVelocityFactor); // meters per second

    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // absolute encoder
    turnConfig.closedLoop.pid(
        MAXSwerveConstants.kTurnP, MAXSwerveConstants.kTurnI, MAXSwerveConstants.kTurnD);
    turnConfig.closedLoop.velocityFF(MAXSwerveConstants.kTurnFF);
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(
        MAXSwerveConstants.kTurnEncoderPositionPIDMinInput,
        MAXSwerveConstants.kTurnEncoderPositionPIDMaxInput);
    turnConfig.closedLoop.outputRange(
        MAXSwerveConstants.kTurnMinOutput, MAXSwerveConstants.kTurnMaxOutput);

    turnConfig.absoluteEncoder.positionConversionFactor(
        MAXSwerveConstants.kTurnEncoderPositionFactor);
    turnConfig.absoluteEncoder.velocityConversionFactor(
        MAXSwerveConstants.kTurnEncoderVelocityFactor);
    turnConfig.absoluteEncoder.inverted(MAXSwerveConstants.kTurnEncoderInverted);

    drivePID = driveMotor.getClosedLoopController();
    turnPID = turnMotor.getClosedLoopController();

    driveConfig.idleMode(IdleMode.kBrake);
    turnConfig.idleMode(IdleMode.kBrake);

    driveConfig.smartCurrentLimit(MAXSwerveConstants.kDriveCurrentLimit);
    turnConfig.smartCurrentLimit(MAXSwerveConstants.kTurnCurrentLimit);

    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Updates the IO */
  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    lastDriveSetpoint = driveSetpoint;

    Logger.recordOutput("DriveFFs/ " + moduleInformation.name, arbFF);

    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMPS = driveEncoder.getVelocity();

    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

    inputs.turnPositionRad = getTurnAngle();
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    inputs.turnError = turnSetpoint - inputs.turnPositionRad.getRadians();
  }

  /** Sets the drive MPS Setpoint */
  @Override
  public void setDriveMPS(double mps) {
    driveSetpoint = mps;
    drivePID.setReference(mps, ControlType.kVelocity);
  }

  /** Sets the turn angle setpoint */
  @Override
  public void setTurnAngle(Rotation2d angle) {
    turnSetpoint = angle.getRadians() - moduleInformation.moduleOffset.getRadians();
    turnPID.setReference(
        angle.getRadians() - moduleInformation.moduleOffset.getRadians(), ControlType.kPosition);
  }

  /** Get the turn angle */
  @Override
  public Rotation2d getTurnAngle() {
    return new Rotation2d(turnEncoder.getPosition() + moduleInformation.moduleOffset.getRadians());
  }
}
