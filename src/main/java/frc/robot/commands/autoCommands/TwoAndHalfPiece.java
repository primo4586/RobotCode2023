// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.commands.actions.AutoCollectCube;
import frc.robot.commands.actions.AutoUpper;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.vision.LimeLight;

public class TwoAndHalfPiece extends SequentialCommandGroup {
  public TwoAndHalfPiece(boolean shouldStartWithCone, boolean areWeBlue, BigArm bigArm,
      LilArm lilArm, Gripper gripper, Swerve swerve, TelescopicArm telescopicArm, LimeLight limeLight) {

    ConditionalCommand putSecondPiece = new ConditionalCommand(
        new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm), () -> shouldStartWithCone);

    ConditionalCommand returnTraj = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath("blueUpperCubeReturn", AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath("redUpperCubeReturn", AutoConstants.pathConstraints), false),
        () -> areWeBlue);

    ConditionalCommand secondDriveBack = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath("blueUpperCubeTwo", AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath("redUpperCubeTwo", AutoConstants.pathConstraints), false),
        () -> areWeBlue);

    ConditionalCommand driveBack = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath(!shouldStartWithCone ? "blueUpperCube" : "blueUpperCone",
            AutoConstants.pathConstraints, false), true),
        swerve.followTrajectory(PathPlanner.loadPath(!shouldStartWithCone ? "redUpperCube" : "redUpperCone",
            AutoConstants.pathConstraints, false), true),
        () -> areWeBlue);

    BooleanSupplier communityClosenesCheck = () -> Math
        .abs(swerve.getPose().getX() - (areWeBlue ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX)) < SwerveConstants.trajAccuracy &&
        Math.abs(swerve.getPose().getY() - 4.42) < SwerveConstants.trajAccuracy;

    ConditionalCommand communityCheck = new ConditionalCommand(Commands.none(),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(
            new Translation2d(areWeBlue ? 1.9 : SwerveConstants.redAligningX, 4.42)), false).asProxy(),
        communityClosenesCheck);

    addCommands(
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(shouldStartWithCone);
        }),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
        new Eject(gripper),
        Commands.runOnce(() -> {
            gripper.setShouldGripCone(false);
          }, gripper),
        driveBack.alongWith(new Ground(gripper, lilArm, bigArm, telescopicArm)),
        returnTraj.alongWith(new AutoUpper(bigArm, lilArm, gripper, telescopicArm)),
        communityCheck,
        putSecondPiece,
        new Eject(gripper),
        secondDriveBack.alongWith(new Ground(gripper, lilArm, bigArm, telescopicArm)),
        //new AutoCollectCube(swerve, gripper, lilArm, bigArm, telescopicArm, limeLight),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint),
        bigArm.TurnBigArmToSetpoint(BigConstants.middleOfRobotSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.middleOfRobotSetPoint),
        new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper));
  }
}
