package frc.robot.commands.actions;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;

import com.pathplanner.lib.PathPoint;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class CoolScore extends SequentialCommandGroup {

  public CoolScore(
      Swerve swerve, BigArm bigArm, LilArm lilArm, Gripper gripper,
      Objective objective, TelescopicArm telescopicArm) {

    // Command to move arms based on the objective node level
    ConditionalCommand moveArms = new ConditionalCommand(
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
        new ConditionalCommand(
            new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
            Commands.none(),
            () -> objective.nodeLevel == NodeLevel.MID),
        () -> objective.nodeLevel == NodeLevel.HYBRID);

    // Calculate end spot translation based on alliance color
    Alliance alliance = DriverStation.getAlliance();
    double aligningX = (alliance == Alliance.Blue) ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX;
    double aligningY = Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]);
    Translation2d endSpotTranslation = new Translation2d(aligningX, aligningY);
    PathPoint endPoint = new PathPoint(endSpotTranslation, new Rotation2d(0));

    Pose2d pose = swerve.getPose();
    PathPoint robotPose = new PathPoint(pose.getTranslation(), swerve.getYaw(),
        swerve.getStates()[0].speedMetersPerSecond);

    List<PathPoint> trajectoryPoints = List.of(robotPose);

    Command lowerDrive = Commands.none();
    Command drive;

    // Determine trajectory based on robot's pose and location
    if (pose.getY() > 3.35) {
      if (pose.getX() > 5.4) {
        trajectoryPoints.addAll(List.of(
            new PathPoint(new Translation2d(5.42, 4.7), new Rotation2d(0)),
            new PathPoint(new Translation2d(2.3, 4.7), new Rotation2d(0)),
            endPoint));
      } else if (pose.getX() > 2.47) {
        trajectoryPoints.addAll(List.of(
            new PathPoint(new Translation2d(2.3, 4.7), new Rotation2d(0)),
            endPoint));
      } else {
        trajectoryPoints.addAll(List.of(endPoint));
      }
    } else {
      if (pose.getX() > 5.4) {
        trajectoryPoints.addAll(List.of(
            new PathPoint(new Translation2d(5.42, 0.9), new Rotation2d(0)),
            new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0))));
        lowerDrive = swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(trajectoryPoints), false)
            .asProxy();
      } else if (pose.getX() > 3.3) {
        trajectoryPoints.addAll(List.of(
            new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0)),
            endPoint));
        lowerDrive = swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(trajectoryPoints), false)
            .asProxy();
      } else if (pose.getX() > 2.47) {
        trajectoryPoints.addAll(List.of(
            new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0)),
            endPoint));
      } else {
        trajectoryPoints.addAll(List.of(endPoint));
      }
    }

    // Drive the robot to the desired location
    drive = swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(trajectoryPoints), false);

    // Check if the robot is close enough to the end spot
    BooleanSupplier closenessCheck = () -> {
      Translation2d currentPos = swerve.getPose().getTranslation();
      return Math.abs(currentPos.getX() - endSpotTranslation.getX()) < SwerveConstants.trajAccuracy &&
          Math.abs(currentPos.getY() - endSpotTranslation.getY()) < SwerveConstants.trajAccuracy;
    };

    // Command to check if the robot has reached the end spot
    ConditionalCommand areWeThere = new ConditionalCommand(
        Commands.none(),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(List.of(endPoint)), false).asProxy(),
        closenessCheck);

    // Execute the sequence of commands
    addCommands(
        lowerDrive,
        new ParallelCommandGroup(
            drive, moveArms),
        areWeThere);
  }
}
