// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.LilArm;

public class MoveArmsToSetPoints extends ParallelCommandGroup {
  public MoveArmsToSetPoints(BigArm bigArm, Double setPointBigArm, LilArm lilArm, Double setPointLilArm) {

  
    addCommands(
      bigArm.TurnBigArmToSetpoint(setPointBigArm),
      lilArm.TurnLilArmToSetpoint(setPointLilArm)
    );
  }
}
