package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ExternalEncoderConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleInformation;
import frc.robot.Constants.MAXSwerveConstants;
import org.littletonrobotics.junction.Logger;

public class MAXSwerveIO_Real implements MAXSwerveIO {

  SparkMax driveMotor;
  SparkMax turnMotor;

  RelativeEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  // Onboard PID controller
  SparkClosedLoopController drivePID;
  SparkClosedLoopController turnPID;

  public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
  public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

  SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  SwerveModuleInformation moduleInformation;

  double turnSetpoint = 0.0;
  double driveSetpoint = 0.0;

  double arbFF = 0.0;

  double lastDriveSetpoint = 0.0;

  public MAXSwerveIO_Real(SwerveModuleInformation moduleInformation) {

    this.moduleInformation = moduleInformation;

    driveMotor = new SparkMax(moduleInformation.driveCAN_ID, MotorType.kBrushless);
    turnMotor = new SparkMax(moduleInformation.turnCAN_ID, MotorType.kBrushless);

    

    //driveMotor.restoreFactoryDefaults();
    //turnMotor.restoreFactoryDefaults();

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(); 
    
    //Type.kDutyCycle? I think this may outdated

    drivePID = driveMotor.getClosedLoopController();
    turnPID = turnMotor.getClosedLoopController();

    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // relative encoder
    driveConfig.closedLoop.pid(MAXSwerveConstants.kDriveP, MAXSwerveConstants.kDriveI, MAXSwerveConstants.kDriveD);
    driveConfig.closedLoop.outputRange(MAXSwerveConstants.kDriveMinOutput, MAXSwerveConstants.kDriveMaxOutput);
    
    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // absolute encoder
    turnConfig.closedLoop.pid(MAXSwerveConstants.kTurnP, MAXSwerveConstants.kTurnI, MAXSwerveConstants.kTurnD);
    turnConfig.closedLoop.velocityFF(MAXSwerveConstants.kTurnFF);
    driveConfig.closedLoop.outputRange(MAXSwerveConstants.kTurnMinOutput, MAXSwerveConstants.kTurnMaxOutput);

    // drivePID.setFeedbackDevice(driveEncoder);
    // turnPID.setFeedbackDevice(turnEncoder);

    driveEncoder.setPositionConversionFactor(MAXSwerveConstants.kDriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(MAXSwerveConstants.kDriveEncoderVelocityFactor);
    driveEncoder.setMeasurementPeriod((int) (1000 / CodeConstants.kMainLoopFrequency));
    driveEncoder.setAverageDepth(2);

    turnEncoder.setPositionConversionFactor(MAXSwerveConstants.kTurnEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(MAXSwerveConstants.kTurnEncoderVelocityFactor);
    turnEncoder.setInverted(MAXSwerveConstants.kTurnEncoderInverted);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(MAXSwerveConstants.kTurnEncoderPositionPIDMinInput);
    turnPID.setPositionPIDWrappingMaxInput(MAXSwerveConstants.kTurnEncoderPositionPIDMaxInput);

    // drivePID.setP(MAXSwerveConstants.kDriveP);
    // drivePID.setI(MAXSwerveConstants.kDriveI);
    // drivePID.setD(MAXSwerveConstants.kDriveD);
    // drivePID.setOutputRange(MAXSwerveConstants.kDriveMinOutput, MAXSwerveConstants.kDriveMaxOutput);

    // turnPID.setP(MAXSwerveConstants.kTurnP);
    // turnPID.setI(MAXSwerveConstants.kTurnI);
    // turnPID.setD(MAXSwerveConstants.kTurnD);
    // turnPID.setFF(MAXSwerveConstants.kTurnFF);
    // turnPID.setOutputRange(MAXSwerveConstants.kTurnMinOutput, MAXSwerveConstants.kTurnMaxOutput);

    driveMotor.setIdleMode(MAXSwerveConstants.kDriveIdleMode);
    turnMotor.setIdleMode(MAXSwerveConstants.kTurnIdleMode);

    driveMotor.setSmartCurrentLimit(MAXSwerveConstants.kDriveCurrentLimit);
    turnMotor.setSmartCurrentLimit(MAXSwerveConstants.kTurnCurrentLimit);

    driveMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000 / CodeConstants.kMainLoopFrequency));
    turnMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000 / CodeConstants.kMainLoopFrequency));


    driveMotor.configure(null, null, null);
    
    driveMotor.burnFlash();
    turnMotor.burnFlash();
  }

  /** Updates the IO */
  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    arbFF =
        driveFeedforward.calculate(
            driveSetpoint, (driveSetpoint - lastDriveSetpoint) * CodeConstants.kMainLoopFrequency);
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
    drivePID.setReference(mps, ControlType.kVelocity, 0, arbFF);
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
