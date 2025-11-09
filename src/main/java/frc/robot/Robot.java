package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MAXSwerveConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.claw.ClawWheels;
import frc.robot.subsystems.arm.claw.ClawWheelsIO_Real;
import frc.robot.subsystems.arm.claw.ClawWheelsIO_Sim;
import frc.robot.subsystems.arm.extension.ArmExtension;
import frc.robot.subsystems.arm.extension.ArmExtensionIO_Real;
import frc.robot.subsystems.arm.extension.ArmExtensionIO_Sim;
import frc.robot.subsystems.arm.pivot.ArmPivot;
import frc.robot.subsystems.arm.pivot.ArmPivotIO_Real;
import frc.robot.subsystems.arm.pivot.ArmPivotIO_Sim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIO_Real;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.MAXSwerveIO_Real;
import frc.robot.subsystems.drive.MAXSwerveIO_Sim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO_Real;
import frc.robot.subsystems.hopper.HopperIO_Sim;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private Superstructure superstructure;
  private MAXSwerve drivebase;
  private ArmExtension armext;
  private ClawWheels claw;
  private ArmPivot pivot;
  private Hopper hopper;

  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  // Control the mode of the robot
  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;

  // Auto Command
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  // Driver Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  public Robot() {

    drivebase =
        new MAXSwerve(
            mode == RobotMode.REAL ? new GyroIO_Real() : new GyroIO() {},
            mode == RobotMode.REAL
                ? new MAXSwerveIO[] {
                  new MAXSwerveIO_Real(DriveConstants.kFrontLeftSwerveModule),
                  new MAXSwerveIO_Real(DriveConstants.kFrontRightSwerveModule),
                  new MAXSwerveIO_Real(DriveConstants.kBackLeftSwerveModule),
                  new MAXSwerveIO_Real(DriveConstants.kBackRightSwerveModule)
                }
                : new MAXSwerveIO[] {
                  new MAXSwerveIO_Sim(),
                  new MAXSwerveIO_Sim(),
                  new MAXSwerveIO_Sim(),
                  new MAXSwerveIO_Sim()
                });
    pivot = new ArmPivot(mode == RobotMode.REAL ? new ArmPivotIO_Real() : new ArmPivotIO_Sim());
    armext =
        new ArmExtension(
            mode == RobotMode.REAL ? new ArmExtensionIO_Real() : new ArmExtensionIO_Sim());
    claw =
        new ClawWheels(mode == RobotMode.REAL ? new ClawWheelsIO_Real() : new ClawWheelsIO_Sim());
    hopper = new Hopper(mode == RobotMode.REAL ? new HopperIO_Real() : new HopperIO_Sim());

    superstructure = new Superstructure(drivebase, armext, claw, pivot, hopper);

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Taxi", superstructure.Taxi());
  }

  @SuppressWarnings(value = "resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("Codebase", "6657 2024");
    switch (mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }
    Logger.start();

    superstructure = new Superstructure(drivebase, armext, claw, pivot, hopper);

    // Set the default command for the drivebase for TeleOP driving
    drivebase.setDefaultCommand(
        drivebase.runVelocityFieldRelative(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.05)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.75,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.15)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.75,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.15)
                        * DriveConstants.kMaxAngularVelocity
                        * 0.5)));

    operator.a().onTrue(superstructure.selectPivotHeight(1));
    operator.b().onTrue(superstructure.selectPivotHeight(2));
    operator.y().onTrue(superstructure.selectPivotHeight(3));
    operator.leftTrigger().onTrue(superstructure.ready());
    operator.rightTrigger().onTrue(superstructure.runIntake()).onFalse(superstructure.stopIntake());
    operator.leftBumper().onTrue(superstructure.homeRobot());

    // operator.a().onTrue(pivot.changeSetpoint(-17));
    // operator.b().onTrue(pivot.changeSetpoint(12));
    // operator.y().onTrue(pivot.changeSetpoint(55));
    // operator.a().onFalse(claw.changeSetpoint(0.75));
    // operator.b().onFalse(pivot.changeSetpoint(-25));
    // operator.y().onFalse(pivot.changeSetpoint(17));

    driver.leftBumper().onTrue(superstructure.homeRobot());
    driver.rightTrigger().onTrue(superstructure.raisePivot());
    driver.leftTrigger().onTrue(superstructure.Score());
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivebase.setPose(
                        new Pose2d(
                            drivebase.getPose().getTranslation(),
                            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero))));

    // Debug Buttons
    // driver.povUp().onTrue(pivot.changeSetpoint(55));
    // driver.povDown().onTrue(pivot.changeSetpoint(-90));

    // driver.a().onTrue(superstructure.extendClaw());
    // driver.b().onTrue(superstructure.retractClaw());
    // driver.y().onTrue(superstructure.ready());

    // driver.a().onTrue(superstructure.selectPivotHeight(1));
    // driver.b().onTrue(superstructure.selectPivotHeight(2));
    // driver.y().onTrue(superstructure.selectPivotHeight(3));

    autoChooser.addOption("Nothing", Commands.print("Nothing Auto Selected"));
    autoChooser.addDefaultOption(
        "Taxi",
        Commands.sequence(drivebase.runVelocity(() -> new ChassisSpeeds(1, 0, 0)).withTimeout(2)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().schedule(autoChooser.get());
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().cancel(autoChooser.get());
      CommandScheduler.getInstance().schedule(drivebase.stop());
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
}
