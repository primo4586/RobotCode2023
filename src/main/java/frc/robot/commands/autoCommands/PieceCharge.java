// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.commands.utils.DriveUntilOtherSide;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class PieceCharge extends SequentialCommandGroup {
  public PieceCharge(boolean shouldStartWithCone, Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, TelescopicArm telescopicArm) {

        Eject eject = new Eject(gripper);

    addCommands(
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldStartWithCone);
        }, gripper),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
        eject,
        new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper),
        new DriveUntilOtherSide(swerve, true),
        new Eject(gripper)
    );
  }
}
