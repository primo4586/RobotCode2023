// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class AlignToTarget extends InstantCommand {
  private Gripper gripper;
  private Swerve swerve;

  public AlignToTarget(Swerve swerve, Gripper gripper) {  
    addRequirements(swerve);
    this.gripper = gripper;
    this.swerve = swerve;
  }

  @Override
  public void initialize( ) {  
    double AligningY;
    int closestXId = swerve.whereToAlign(gripper);
    Rotation2d aligningRotation = new Rotation2d(swerve.getYaw().getDegrees());
    Pose2d alignLocation;


    AligningY = SwerveConstants.aligningY;

    if(gripper.getShouldGripCone())
      alignLocation = new Pose2d(SwerveConstants.coneAligningX[closestXId], AligningY, aligningRotation);
    else
      alignLocation = new Pose2d(SwerveConstants.cubeAligningX[closestXId], AligningY, aligningRotation);
    swerve.followTrajectory(swerve.generateTrajectoryToGoal(alignLocation.getTranslation()),false, true );
  }
}
