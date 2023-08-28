// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import java.util.List;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class CoolScoreDrive extends InstantCommand {
  /** Creates a new CoolScoreDrive. */
  private Swerve swerve;
  private Objective objective;
  private Alliance alliance = DriverStation.getAlliance();

  PathPoint blueUpperAlignToEnterCommunity = new PathPoint(new Translation2d(5.42, 4.7), new Rotation2d(0));
  PathPoint blueUpperEnterCommunity = new PathPoint(new Translation2d(2.3, 4.7), new Rotation2d(0));
  
  PathPoint blueLowerAlignToEnterCommunity = new PathPoint(new Translation2d(5.42, 0.9), new Rotation2d(0));
  PathPoint blueLowerEnterCommunity = new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0));

  public CoolScoreDrive(Swerve swerve, Objective objective) {
    this.swerve = swerve;
    this.objective = objective;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double aligningX = (alliance == Alliance.Blue) ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX;
    double aligningY = Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]);
    Translation2d endPointTranslatio2d = new Translation2d(aligningX, aligningY);
    PathPoint endPoint = new PathPoint(endPointTranslatio2d, new Rotation2d(0));

    Pose2d pose = swerve.getPose();
    PathPoint robotPose = new PathPoint(
        pose.getTranslation(), swerve.getYaw(), swerve.getStates()[0].speedMetersPerSecond);

    List<PathPoint> pointsList = null;

    if(!(Math.abs(swerve.getPose().getX() - endPointTranslatio2d.getX()) > SwerveConstants.trajAccuracy ||
        Math.abs(swerve.getPose().getY() - endPointTranslatio2d.getY()) > SwerveConstants.trajAccuracy)){
      if (alliance == Alliance.Blue) {
        if (pose.getY() > 3.35) {
          if (pose.getX() > 5.42) {
            pointsList = List.of(robotPose, blueUpperAlignToEnterCommunity, blueUpperEnterCommunity, endPoint);
          } else if (pose.getX() > 2.47) {
            pointsList = List.of(robotPose, blueUpperEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        } else {
          if (pose.getX() > 5.42) {
            pointsList = List.of(robotPose, blueLowerAlignToEnterCommunity, blueLowerEnterCommunity);
          } else if (pose.getX() > 3.3) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity);
          } else if (pose.getX() > 2.47) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        }
      } else {
        if (pose.getY() > 3.35) {
          if (pose.getX() > 11.13) {
            pointsList = List.of(robotPose, blueUpperAlignToEnterCommunity, blueUpperEnterCommunity, endPoint);
          } else if (pose.getX() > 14.08) {
            pointsList = List.of(robotPose, blueUpperEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        } else {
          if (pose.getX() > 11.13) {
            pointsList = List.of(robotPose, blueLowerAlignToEnterCommunity, blueLowerEnterCommunity);
          } else if (pose.getX() > 13.25) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity);
          } else if (pose.getX() > 14.08) {
            pointsList = List.of(robotPose, blueLowerEnterCommunity, endPoint);
          } else {
            pointsList = List.of(robotPose, endPoint);
          }
        }
      }

      swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(pointsList), false);
      if (Math.abs(swerve.getPose().getX() - endPointTranslatio2d.getX()) > SwerveConstants.trajAccuracy ||
          Math.abs(swerve.getPose().getY() - endPointTranslatio2d.getY()) > SwerveConstants.trajAccuracy) {
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(endPointTranslatio2d), false);
      }
      if (Math.abs(swerve.getPose().getX() - endPointTranslatio2d.getX()) > SwerveConstants.trajAccuracy ||
          Math.abs(swerve.getPose().getY() - endPointTranslatio2d.getY()) > SwerveConstants.trajAccuracy) {
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(endPointTranslatio2d), false);
      }
      if (Math.abs(swerve.getPose().getX() - endPointTranslatio2d.getX()) > SwerveConstants.trajAccuracy ||
          Math.abs(swerve.getPose().getY() - endPointTranslatio2d.getY()) > SwerveConstants.trajAccuracy) {
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(endPointTranslatio2d), false);
      }
    }
    
  }
}
