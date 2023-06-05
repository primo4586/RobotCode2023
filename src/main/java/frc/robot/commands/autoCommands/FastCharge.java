// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
public class FastCharge extends SequentialCommandGroup {
  public FastCharge(Boolean shouldDriveBack ,Swerve swerve) {
    addCommands(
    new maybeBetterCharge(shouldDriveBack, swerve),
    new PIDBalance(swerve)
    );
  }
}
