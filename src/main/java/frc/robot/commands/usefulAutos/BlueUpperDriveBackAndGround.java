// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.usefulAutos;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.commands.actions.GroundAuto;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
public class BlueUpperDriveBackAndGround extends ParallelCommandGroup {
  public BlueUpperDriveBackAndGround(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, boolean drivingFromCube) {

    addCommands(
      new GroundAuto(gripper, lilArm, bigArm),
      swerve.followTrajectory(PathPlanner.loadPath(drivingFromCube ? "blueUpperCube" : "blueUpperCone", Constants.AutoConstants.pathConstraints, false), true)
    );
  }
}
