// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.actions.GroundAuto;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
public class DriveBackAndGround extends ParallelCommandGroup {
  public DriveBackAndGround(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm) {
    addCommands(
      Commands.waitSeconds(2.0).andThen(new GroundAuto(gripper, lilArm, bigArm)),
      swerve.followTrajectory(PathPlanner.loadPath("upperCube2", Constants.AutoConstants.pathConstraints, false), true)
    );
  }
}