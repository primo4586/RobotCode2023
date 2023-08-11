// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.autoCommands.utils.GroundAuto;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class Bump2Piece extends SequentialCommandGroup {
  public Bump2Piece(boolean shouldStartWithCone, boolean shouldFinishWithCone, boolean areWeBlue, BigArm bigArm,
      LilArm lilArm, Gripper gripper, Swerve swerve) {

    ConditionalCommand driveBack = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath(shouldStartWithCone ? "blueLowerCone" : "blueLowerCube",
            AutoConstants.pathConstraints), false),
        swerve.followTrajectory(
            PathPlanner.loadPath(shouldStartWithCone ? "redLowerCone" : "redLowerCube", AutoConstants.pathConstraints),
            false),
        () -> areWeBlue);

    ConditionalCommand putSecondPiece = new ConditionalCommand(new PutItemInTheMiddle(lilArm, bigArm, gripper),
        new PutItemInTheUpper(bigArm, lilArm, gripper), () -> shouldStartWithCone == shouldFinishWithCone);

    ConditionalCommand returnTraj = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath("blueLowerReturn", AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath("redLowerReturn", AutoConstants.pathConstraints), false),
        () -> areWeBlue);

    addCommands(

        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldStartWithCone);
        }, gripper),

        new PutItemInTheUpper(bigArm, lilArm, gripper),
        Commands.waitSeconds(0.4),
        gripper.getEjectCommand(),
        Commands.waitSeconds(0.3),
        lilArm.closeLilArmSolenoid(),
        driveBack
            .alongWith(new GroundAuto(gripper, lilArm, bigArm, false)), // TODO: fix Ground setPoints
        lilArm.openLilArmSolenoid(),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(
                areWeBlue ? new Translation2d(6.55, 0.92) : new Translation2d(9.91, 0.92)), false)
            .alongWith(Commands.waitSeconds(0.2).andThen(gripper.getCollectCommand())),

        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldFinishWithCone);
        }, gripper),
        returnTraj
            .alongWith(new PutItemInTheUpper(bigArm, lilArm, gripper)),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(
            areWeBlue ? new Translation2d(1.90, 1.07) : new Translation2d(14.65, 1.07)), areWeBlue),
        putSecondPiece,
        Commands.waitSeconds(0.4),
        gripper.getEjectCommand(),
        Commands.waitSeconds(0.4),
        new IntakeSequential(lilArm, bigArm));
  }
}
