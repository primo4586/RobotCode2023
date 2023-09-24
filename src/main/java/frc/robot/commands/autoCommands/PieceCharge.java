// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.GroundOnlyArms;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.commands.utils.DriveUntilOtherSide;
import frc.robot.commands.utils.FastCharge;
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
        new GroundOnlyArms(lilArm, bigArm, telescopicArm),//TODO: test if possible to do this while driving back
        new DriveUntilOtherSide(swerve, true),
        new Ground(gripper, lilArm, bigArm, telescopicArm),
        new FastCharge(false, swerve)

    );
  }
}
