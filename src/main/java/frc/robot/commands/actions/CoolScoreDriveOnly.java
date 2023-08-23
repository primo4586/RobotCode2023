package frc.robot.commands.actions;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class CoolScoreDriveOnly extends ParallelCommandGroup {
  public CoolScoreDriveOnly(Swerve swerve,BigArm bigArm, LilArm lilArm, Gripper gripper , Objective objective, TelescopicArm telescopicArm) {

    Translation2d robotEndSpot;

    if (DriverStation.getAlliance() == Alliance.Blue)
      robotEndSpot = new Translation2d(SwerveConstants.blueAligningX,
          Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]));
    else
      robotEndSpot = new Translation2d(SwerveConstants.redAligningX,
          Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]));
    // Generate trajectory to alignment pose based on alliance color
    PathPlannerTrajectory trajectory = swerve.generateTrajectoryToAligmentPose(robotEndSpot);

    BooleanSupplier closenesCheck = ()->Math.abs(swerve.getPose().getX() - robotEndSpot.getX()) < SwerveConstants.trajAccuracy &&
        Math.abs(swerve.getPose().getY() - robotEndSpot.getY()) < SwerveConstants.trajAccuracy;

    ConditionalCommand areWeThere = new ConditionalCommand(Commands.none(), 
        swerve.followTrajectory(trajectory, false).asProxy(),
        closenesCheck);

    addCommands(
        swerve.followTrajectory(trajectory, false).asProxy(),
        areWeThere);
  }
}
