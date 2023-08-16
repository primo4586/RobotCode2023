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
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.GroundOnlyArms;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class Bump2Piece extends SequentialCommandGroup {
  public Bump2Piece(boolean shouldStartWithCone, boolean shouldFinishWithCone, boolean areWeBlue, BigArm bigArm,
      LilArm lilArm, Gripper gripper, Swerve swerve, TelescopicArm telescopicArm) {

    ConditionalCommand driveBack = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath(shouldStartWithCone ? "blueLowerCone" : "blueLowerCube",
            AutoConstants.pathConstraints), false),
        swerve.followTrajectory(
            PathPlanner.loadPath(shouldStartWithCone ? "redLowerCone" : "redLowerCube", AutoConstants.pathConstraints),
            false),
        () -> areWeBlue);

    ConditionalCommand putSecondPiece = new ConditionalCommand(new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm), () -> shouldStartWithCone == shouldFinishWithCone);

    ConditionalCommand returnTraj = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath("blueLowerReturn", AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath("redLowerReturn", AutoConstants.pathConstraints), false),
        () -> areWeBlue);

    addCommands(
        Commands.runOnce(()->gripper.setShouldGripCone(shouldFinishWithCone),gripper),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
        gripper.getEjectCommand(),
            Commands.waitSeconds(0.3),
        new GroundOnlyArms(gripper, lilArm, bigArm, telescopicArm),
            driveBack,
        new Ground(gripper, lilArm, bigArm, telescopicArm),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(
                areWeBlue ? new Translation2d(6.55, 0.92) : new Translation2d(9.91, 0.92)), false)
            .alongWith(Commands.waitSeconds(0.2).andThen(gripper.getCollectCommand())),//TODO: test otf traj and maybe change it
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldFinishWithCone);
        }, gripper),
        returnTraj
            .alongWith(new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(
            areWeBlue ? new Translation2d(1.90, 1.07) : new Translation2d(14.65, 1.07)), false),
        putSecondPiece,
        gripper.getEjectCommand(),
        Commands.waitSeconds(0.2),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint),
        bigArm.TurnBigArmToSetpoint(BigConstants.middleOfRobotSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.middleOfRobotSetPoint));
  }
}
