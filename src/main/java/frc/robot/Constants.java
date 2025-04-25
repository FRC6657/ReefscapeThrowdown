package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final class CodeConstants {
    public static final double kMainLoopFrequency = 50; // Hz
  }

  public static final class CANID {

    public static final int kFrontLeftDrive = 1;
    public static final int kBackLeftDrive = 3;
    public static final int kFrontRightDrive = 2;
    public static final int kBackRightDrive = 4;

    public static final int kFrontLeftTurn = 5;
    public static final int kBackLeftTurn = 7;
    public static final int kFrontRightTurn = 6;
    public static final int kBackRightTurn = 8;

    public static final int kPigeon = 9;
  }

  public static final class AutoConstants {

    // Choreo
    public static final PIDController kXController = new PIDController(9, 0, 0);
    public static final PIDController kYController = new PIDController(9, 0, 0);
    public static final PIDController kThetaController = new PIDController(8, 0, 0);

    // Auto Align
    public static final double kAA_P_X = 3.8;
    public static final double kAA_P_Y = 3.8;
    public static final double kAA_P_Theta = 8;

    public static final double kAA_T_Clamp = 3; // m/s
    public static final double kAA_R_Clamp = 2 * Math.PI; // rad/s

    public static final double kAA_T_Tolerance = Units.inchesToMeters(2); // m
    public static final double kAA_R_Tolerance = Units.degreesToRadians(2); // rad

    // Note Aim
    public static final double kNA_P = 1d / 17;

    // Starting Positions
    public static final Pose2d BLUE_CENTER_FENDER =
        new Pose2d(new Translation2d(1.375, 5.55), new Rotation2d(0));
    public static final Pose2d RED_CENTER_FENDER =
        new Pose2d(new Translation2d(15.175, 5.55), new Rotation2d(Math.PI));
    public static final Pose2d BLUE_AMP_FENDER = new Pose2d(0.695, 6.735, new Rotation2d(1.05));
    public static final Pose2d RED_AMP_FENDER = new Pose2d(15.875, 6.735, new Rotation2d(2.091));
    public static final Pose2d BLUE_SOURCE_FENDER = new Pose2d(0.695, 4.385, new Rotation2d(-1.05));
    public static final Pose2d RED_SOURCE_FENDER =
        new Pose2d(15.875, 4.385, new Rotation2d(-2.091));

    public static final Pose2d BLUE_SOURCE_AUTO_START =
        new Pose2d(1.216, 3.960, new Rotation2d(-0.963));
    public static final Pose2d RED_SOURCE_AUTO_START =
        new Pose2d(15.3, 3.960, new Rotation2d(-2.179));
  }

  public static final class DriveConstants {

    public static final class SwerveModuleInformation {
      public final String name;
      public final int driveCAN_ID;
      public final int turnCAN_ID;
      public final Rotation2d moduleOffset;

      public SwerveModuleInformation(
          String name, int driveCAN_ID, int turnCAN_ID, Rotation2d moduleOffset) {
        this.name = name;
        this.driveCAN_ID = driveCAN_ID;
        this.turnCAN_ID = turnCAN_ID;
        this.moduleOffset = moduleOffset;
      }
    }

    // Define Swerve Modules
    public static final SwerveModuleInformation kFrontLeftSwerveModule =
        new SwerveModuleInformation(
            "Front Left", CANID.kFrontLeftDrive, CANID.kFrontLeftTurn, new Rotation2d(Math.PI / 2));
    public static final SwerveModuleInformation kBackLeftSwerveModule =
        new SwerveModuleInformation(
            "Back Left", CANID.kBackLeftDrive, CANID.kBackLeftTurn, new Rotation2d(Math.PI));
    public static final SwerveModuleInformation kFrontRightSwerveModule =
        new SwerveModuleInformation(
            "Front Right", CANID.kFrontRightDrive, CANID.kFrontRightTurn, new Rotation2d(0));
    public static final SwerveModuleInformation kBackRightSwerveModule =
        new SwerveModuleInformation(
            "Back Right",
            CANID.kBackRightDrive,
            CANID.kBackRightTurn,
            new Rotation2d(-Math.PI / 2));

    public static final SwerveModuleInformation[] kIndexedSwerveModuleInformation = {
      kFrontLeftSwerveModule, kBackLeftSwerveModule, kFrontRightSwerveModule, kBackRightSwerveModule
    };

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);

    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    public static final Translation2d[] kModuleLocations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public static double kDrivebaseRadius = Math.hypot(kWheelBase, kTrackWidth) / 2;

    public static final double kMaxAngularVelocity =
        MAXSwerveConstants.kMaxDriveSpeed / kDrivebaseRadius;

    public static final double kSlowSpeed = 0.5;

    public static final double kS = 0.1;
    public static final double kV = 12d / MAXSwerveConstants.kMaxDriveSpeed; // v/m/s
    public static final double kA = 0.45; // Accel/V
  }

  public static final class MAXSwerveConstants {

    public static enum DriveRatio {
      LOW(12, 22d),
      MEDIUM(13, 22d),
      HIGH(14, 22d),
      EH1(14, 21d),
      EH2(14, 20d),
      EH3(15, 20d),
      EH4(16, 20d),
      EH5(16, 19d),
      ;

      public final double ratio;

      DriveRatio(int pinionTeeth, double spurTeeth) {
        this.ratio = spurTeeth / pinionTeeth;
      }
    }

    public static final double kDriveMotorRatio = DriveRatio.HIGH.ratio;

    // Invert the Turn encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurnEncoderInverted = true;

    // Calculations required for Drive motor conversion factors and feed forward
    public static final double kDriveMotorFreeSpeedRps =
        Units.radiansPerSecondToRotationsPerMinute(DCMotor.getNEO(1).freeSpeedRadPerSec) / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the
    // bevel pinion
    public static final double kDriveMotorReduction = (45.0 / 15) * kDriveMotorRatio;
    public static final double kMaxDriveSpeed =
        (kDriveMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDriveMotorReduction;

    public static final double kDriveEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDriveMotorReduction; // meters
    public static final double kDriveEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDriveMotorReduction) / 60.0; // meters per second

    public static final double kTurnMotorReduction = 9424d / 203;

    public static final double kTurnEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurnEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurnEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurnEncoderPositionPIDMaxInput =
        kTurnEncoderPositionFactor; // radians

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 1 / kMaxDriveSpeed;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final double kTurnP = 1.1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;

    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurnIdleMode = IdleMode.kBrake;

    public static final int kDriveCurrentLimit = 40; // amps
    public static final int kTurnCurrentLimit = 20; // amps
  }
}
