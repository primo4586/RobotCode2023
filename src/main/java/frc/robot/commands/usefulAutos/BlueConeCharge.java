// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.usefulAutos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.GroundAuto;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.usefulAutos.utils.DriveUntilOtherSide;
import frc.robot.commands.usefulAutos.utils.FastCharge;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class BlueConeCharge extends SequentialCommandGroup {
  public BlueConeCharge(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, boolean UpperPiece) {

    ConditionalCommand driveToPiece = new ConditionalCommand(
        swerve.followTrajectory(new Swerve().generateTrajectoryToAligmentPose(new Translation2d(6.65, 3.357626)),
            false),
        swerve.followTrajectory(new Swerve().generateTrajectoryToAligmentPose(new Translation2d(6.65, 2.138426)),
            false),
        () -> UpperPiece);

    addCommands(
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(true);
          gripper.turnOnLed();
        }, gripper),

        new PutItemInTheUpper(bigArm, lilArm, gripper),
        new GroundAuto(gripper, lilArm, bigArm, false),//TODO: test if possible to do this while driving back
        new DriveUntilOtherSide(swerve), // TODO: fix DriveUntilOtherSide
        swerve.followTrajectory(new Swerve().generateTrajectoryToAligmentPose(new Translation2d(6.65, 2.138426)),
            false),
        new FastCharge(false, swerve)

    );
  }
}
