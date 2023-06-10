// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.autoCommands.utils.DriveUntilOtherSide;
import frc.robot.commands.autoCommands.utils.FastCharge;
import frc.robot.commands.autoCommands.utils.GroundAuto;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class PieceReloadCharge extends SequentialCommandGroup {
  public PieceReloadCharge(boolean shouldStartWithCone, boolean UpperPiece, boolean areWeBlue, Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm) {

    ConditionalCommand driveToPiece = new ConditionalCommand(
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(new Translation2d(6.65, 3.357626)),
            false),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(UpperPiece ? new Translation2d(6.65, 2.138426) : new Translation2d(6.65, 3.357626)),
            false),
        () -> areWeBlue);

    addCommands(
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldStartWithCone);
          gripper.turnOnLed();
        }, gripper),

        new PutItemInTheUpper(bigArm, lilArm, gripper),
        new GroundAuto(gripper, lilArm, bigArm, false),//TODO: test if possible to do this while driving back
        new DriveUntilOtherSide(swerve), // TODO: fix DriveUntilOtherSide
        driveToPiece.alongWith(lilArm.openLilArmSolenoid()),
        new FastCharge(false, swerve)

    );
  }
}
