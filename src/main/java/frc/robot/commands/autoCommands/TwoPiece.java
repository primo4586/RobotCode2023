// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.autoCommands.utils.DriveBackAndGround;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class TwoPiece extends SequentialCommandGroup {
  public TwoPiece(boolean shouldStartWithCone, boolean shouldFinishWithCone, boolean areWeBlue, BigArm bigArm, LilArm lilArm,Gripper gripper,Swerve swerve) {

    DriveBackAndGround driveBackAndGround = new DriveBackAndGround(swerve, gripper, bigArm, lilArm, !shouldStartWithCone, areWeBlue);

    ConditionalCommand putSecondPiece = new ConditionalCommand(new PutItemInTheMiddle(lilArm, bigArm, gripper),
        new PutItemInTheUpper(bigArm, lilArm, gripper), () -> shouldStartWithCone == shouldFinishWithCone);

    ConditionalCommand returnTraj = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath(shouldFinishWithCone ? "blueUpperConeReturn" : "blueUpperCubeReturn",AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath(shouldFinishWithCone ? "redUpperConeReturn" : "redUpperCubeReturn",AutoConstants.pathConstraints), false),
        () -> areWeBlue);

    addCommands(
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldStartWithCone);
          gripper.turnOnLed();
        }, gripper),

        new PutItemInTheUpper(bigArm, lilArm, gripper),
        Commands.waitSeconds(0.4),
        gripper.openGripper(),
        Commands.waitSeconds(0.3),
        lilArm.closeLilArmSolenoid(),
        driveBackAndGround, // TODO: fix Ground setPoints
        Commands.waitSeconds(0.2),
        gripper.closeGripper(),
        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldFinishWithCone);
          gripper.turnOnLed();
        }, gripper),

        returnTraj
            .alongWith(new PutItemInTheUpper(bigArm, lilArm, gripper)),
        putSecondPiece,
        Commands.waitSeconds(0.4),
        gripper.openGripper(),
        Commands.waitSeconds(0.4),
        new IntakeSequential(lilArm, bigArm).alongWith(Commands.waitSeconds(0.2).andThen(gripper.closeGripper())));
  }
}
