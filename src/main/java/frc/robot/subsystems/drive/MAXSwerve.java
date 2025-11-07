package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MAXSwerveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MAXSwerve extends SubsystemBase {

  // Gryo IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Swerve Kinematics
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.kModuleLocations);

  // Odometry
  private SwerveDrivePoseEstimator poseEstimator;

  // Array of swerve modules
  private final MAXSwerveModule[] modules;

  // Track the last heading for when the gyro is disconnected/simulated
  private Rotation2d lastHeading = new Rotation2d();

  /**
   * Creates a new MAXSwerve drivebase
   *
   * @param gyroIO GyroIO
   * @param MAXSwerveIOs Array of MAXSwerveIOs
   */
  public MAXSwerve(GyroIO gyroIO, MAXSwerveIO[] MAXSwerveIOs) {

    // Set gyroIO
    this.gyroIO = gyroIO;

    // Create array of swerve modules
    modules = new MAXSwerveModule[MAXSwerveIOs.length];

    // Create a swerve module for each MAXSwerveIO
    for (int i = 0; i < MAXSwerveIOs.length; i++) {
      modules[i] =
          new MAXSwerveModule(
              MAXSwerveIOs[i], DriveConstants.kIndexedSwerveModuleInformation[i].name + " Module");
    }

    // Create the pose estimator
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(gyroInputs.yawPosition),
            getModulePositions(),
            new Pose2d());
  }

  /** This code runs at 50hz and is responsible for updating the IO and pose estimator */
  @Override
  public void periodic() {

    lastHeading = getPose().getRotation();

    // Update Swerve Module Inputs
    for (var module : modules) {
      module.updateInputs();
    }
    gyroIO.updateInputs(gyroInputs);

    Logger.processInputs("Gyro", gyroInputs);

    var gyroDelta =
        new Rotation2d(
            kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                * 1
                / CodeConstants.kMainLoopFrequency);

    lastHeading = lastHeading.plus(gyroDelta);

    if (RobotBase.isReal()) {
      poseEstimator.update(Rotation2d.fromDegrees(gyroInputs.yawPosition), getModulePositions());
    } else {
      poseEstimator.update(lastHeading, getModulePositions());
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }
  }

  /**
   * Runs the drivebase using a continuous Chassis Speed Input
   *
   * @param speeds Continuous Chassis Speed Input
   * @return Command that runs the drivebase
   */
  public Command runVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          this.runChassisSpeeds(speeds.get());
        });
  }

  /*
   * Stops the drivebase
   */
  public Command stop() {
    return runVelocity(() -> new ChassisSpeeds());
  }

  /**
   * Runs the drivebase using a continuous Chassis Speed Input in field relative mode
   *
   * @param speeds Continuous Chassis Speed Input
   * @return Command that runs the drivebase in field relative mode
   */
  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocity(
        () ->
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.get(), getPose().getRotation().plus(new Rotation2d(isRed() ? Math.PI : 0))));
  }

  private boolean isRed() {
    boolean isRed = false;
    if (DriverStation.getAlliance().isPresent()) {
      isRed = (DriverStation.getAlliance().get() == Alliance.Red);
    }
    return isRed;
  }

  /**
   * Runs the drivebase directly using a chassis speed input
   *
   * @param speeds desired chassis speed
   */
  public void runChassisSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 1 / CodeConstants.kMainLoopFrequency);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAXSwerveConstants.kMaxDriveSpeed);

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      optimizedSetpointStates[i] = modules[i].run(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getSwerveModuleState();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveStates/ChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  // Returns the distance and angle of each module
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getSwerveModulePosition();
    }
    return positions;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (RobotBase.isReal()) {
      poseEstimator.resetPosition(
          Rotation2d.fromDegrees(gyroInputs.yawPosition), getModulePositions(), pose);
    } else {
      poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }
  }

  public boolean chasePose(Pose2d destinationPose) {

    Pose2d currentPose = getPose();

    double xError = destinationPose.getTranslation().minus(currentPose.getTranslation()).getX();
    double yError = destinationPose.getTranslation().minus(currentPose.getTranslation()).getY();
    double thetaError = destinationPose.getRotation().minus(currentPose.getRotation()).getRadians();

    boolean atX = MathUtil.isNear(0, xError, AutoConstants.kAA_T_Tolerance);
    boolean atY = MathUtil.isNear(0, yError, AutoConstants.kAA_T_Tolerance);
    boolean atTheta = MathUtil.isNear(0, thetaError, AutoConstants.kAA_R_Tolerance);

    double xSpeed =
        MathUtil.clamp(
            xError * AutoConstants.kAA_P_X, -AutoConstants.kAA_T_Clamp, AutoConstants.kAA_T_Clamp);
    double ySpeed =
        MathUtil.clamp(
            yError * AutoConstants.kAA_P_Y, -AutoConstants.kAA_T_Clamp, AutoConstants.kAA_T_Clamp);
    double thetaSpeed =
        MathUtil.clamp(
            thetaError * AutoConstants.kAA_P_Theta,
            -AutoConstants.kAA_R_Clamp,
            AutoConstants.kAA_R_Clamp);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), currentPose.getRotation());

    Logger.recordOutput("AutoAim/DesiredShotPos", destinationPose);
    Logger.recordOutput("AutoAim/X Error", xError);
    Logger.recordOutput("AutoAim/Y Error", yError);
    Logger.recordOutput("AutoAim/Theta Error", thetaError);

    Logger.recordOutput("AutoAim/atX", atX);
    Logger.recordOutput("AutoAim/atY", atY);
    Logger.recordOutput("AutoAim/atTheta", atTheta);

    this.runChassisSpeeds(speeds);

    return (atX && atY && atTheta);
  }

  public Command goToPose(Pose2d targetPose) {
    return this.run(() -> {}).until(() -> chasePose(targetPose));
  }
}
