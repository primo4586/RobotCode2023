package frc.robot.commands.actions;

import java.util.function.BooleanSupplier;
import java.util.List;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;

import com.pathplanner.lib.PathPoint;

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

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class CoolScoreDriveOnly extends ParallelCommandGroup {

  public CoolScoreDriveOnly(Swerve swerve, Objective objective) {
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

    drive = swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(trajectoryPoints), false);

    BooleanSupplier closenessCheck = () -> {
      Translation2d currentPos = swerve.getPose().getTranslation();
      return Math.abs(currentPos.getX() - endSpotTranslation.getX()) < SwerveConstants.trajAccuracy &&
          Math.abs(currentPos.getY() - endSpotTranslation.getY()) < SwerveConstants.trajAccuracy;
    };

    ConditionalCommand areWeThere = new ConditionalCommand(
        Commands.none(),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(List.of(endPoint)), false).asProxy(),
        closenessCheck);

    addCommands(
        lowerDrive,
        drive,
        areWeThere);
  }
}
