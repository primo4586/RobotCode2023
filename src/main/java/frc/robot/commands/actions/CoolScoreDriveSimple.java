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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoolScoreDriveSimple extends InstantCommand {
  private Swerve swerve;
  private Objective objective;
  private Alliance alliance = DriverStation.getAlliance();

  public CoolScoreDriveSimple(Swerve swerve, Objective objective) {
    this.swerve = swerve;
    this.objective = objective;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance();
    double aligningX = (alliance == Alliance.Blue) ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX;
    double aligningY = Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]);
    Translation2d endPointTranslatio2d = new Translation2d(aligningX, aligningY);
    PathPoint endPoint = new PathPoint(endPointTranslatio2d, new Rotation2d(Units.degreesToRadians(180)));

    Pose2d pose = swerve.getPose();
    PathPoint robotPose = new PathPoint(
        pose.getTranslation(), swerve.getYaw(), swerve.getStates()[0].speedMetersPerSecond);
    
    List<PathPoint> pointsList = null;
        
    pointsList = List.of(robotPose, endPoint);

    swerve.followTrajectory(swerve.generateTrajectoryToPoseList(pointsList), false).schedule();
    if (Math.abs(swerve.getPose().getX() - endPointTranslatio2d.getX()) > SwerveConstants.trajAccuracy ||
        Math.abs(swerve.getPose().getY() - endPointTranslatio2d.getY()) > SwerveConstants.trajAccuracy) {
          swerve.followTrajectory(swerve.generateTrajectoryToPoseList(pointsList), false).schedule();
    }
  }
}
