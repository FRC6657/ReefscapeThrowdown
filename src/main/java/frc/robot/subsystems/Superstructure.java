package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.arm.claw.ClawWheels;
import frc.robot.subsystems.arm.extension.ArmExtension;
import frc.robot.subsystems.arm.pivot.ArmPivot;
import frc.robot.subsystems.drive.MAXSwerve;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure {

  private MAXSwerve drivebase;
  private ArmExtension armext;
  private ClawWheels claw;
  private ArmPivot pivot;

  private String extentionState = "retracted";

  public Superstructure(MAXSwerve drivebase, ArmExtension armext, ClawWheels claw, ArmPivot pivot) {
    this.drivebase = drivebase;
    this.armext = armext;
    this.claw = claw;
    this.pivot = pivot;
  }

  @AutoLogOutput(key = "Superstructure/RobotPoses")
  public Pose3d[] getRobotPoses() {
    double fakeExtentionDistance = extentionState == "extended" ? 3.25 : 0;
    return new Pose3d[] {
      new Pose3d(
          0.184150,
          0,
          0.822649,
          new Rotation3d(0, -Units.degreesToRadians(pivot.getPosition()) - Math.PI / 2, 0.0)),
      new Pose3d(
          0.184150
              + Math.cos(Units.degreesToRadians(pivot.getPosition()))
                  * Units.inchesToMeters(fakeExtentionDistance),
          0,
          0.822649
              + Math.sin(Units.degreesToRadians(pivot.getPosition()))
                  * Units.inchesToMeters(fakeExtentionDistance),
          new Rotation3d(0, -Units.degreesToRadians(pivot.getPosition()) - Math.PI / 2, 0))
    };
  }
}
