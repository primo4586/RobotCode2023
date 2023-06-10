// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands.utils;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
public class DriveBackAndGround extends ParallelCommandGroup {
  public DriveBackAndGround(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, boolean drivingFromCube, boolean areWeBlue) {

    addCommands(
      new GroundAuto(gripper, lilArm, bigArm, true),
      new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath(drivingFromCube ? "blueUpperCube" : "blueUpperCone", Constants.AutoConstants.pathConstraints, false), true),
        swerve.followTrajectory(PathPlanner.loadPath(drivingFromCube ? "redUpperCube" : "redUpperCone", Constants.AutoConstants.pathConstraints, false), true),
        ()->areWeBlue) 
    );
  }
}
