// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.utils.DriveToCube;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

public class AutoCollectCube extends SequentialCommandGroup {
  public AutoCollectCube(Swerve swerve, Gripper gripper, LilArm lilArm, BigArm bigArm, TelescopicArm telescopicArm) {
    Ground ground = new Ground(gripper, lilArm, bigArm, telescopicArm);
    DriveToCube driveToCube = new DriveToCube(swerve,gripper);
    addCommands(
      ground,
      driveToCube
    );
  }
}
