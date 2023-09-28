// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.GroundOnlyArms;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.vision.LimeLight;

public class Bump2Piece extends SequentialCommandGroup {
  public Bump2Piece(boolean shouldStartWithCone, boolean areWeBlue, BigArm bigArm,
      LilArm lilArm, Gripper gripper, Swerve swerve, TelescopicArm telescopicArm, LimeLight limeLight) {

    ConditionalCommand driveBack = new ConditionalCommand(//TODO: make sure all the paths end with vel overide
        swerve.followTrajectory(PathPlanner.loadPath(shouldStartWithCone ? "blueLowerCone" : "blueLowerCube",
            AutoConstants.pathConstraints), false),
        swerve.followTrajectory(
            PathPlanner.loadPath(shouldStartWithCone ? "redLowerCone" : "redLowerCube", AutoConstants.pathConstraints),
            false),
        () -> areWeBlue);

    ConditionalCommand returnTraj = new ConditionalCommand(
        swerve.followTrajectory(PathPlanner.loadPath("blueLowerCubeReturn", AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath("redLowerCubeReturn", AutoConstants.pathConstraints), false),
            () -> areWeBlue);
        
    BooleanSupplier communityClosenesCheck = () -> Math
            .abs(swerve.getPose().getX() - (areWeBlue ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX)) < SwerveConstants.trajAccuracy &&
            Math.abs(swerve.getPose().getY() - 1.07) < SwerveConstants.trajAccuracy;

    ConditionalCommand communityCheck = new ConditionalCommand(Commands.none(),
            swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(areWeBlue ? new Translation2d(SwerveConstants.blueAligningX, 1.07) : new Translation2d(SwerveConstants.redAligningX, 1.07)),false).asProxy(),
            communityClosenesCheck);
        
    Command driveToCollect = swerve.followTrajectory(swerve.generateTrajectoryToPoseList(
            List.of(new PathPoint(swerve.getPose().getTranslation(), swerve.getPose().getRotation(),swerve.getStates()[0].speedMetersPerSecond),
            new PathPoint(new Translation2d(areWeBlue ? 6.8:9.75, 0.92), new Rotation2d(0)))),false);

    ConditionalCommand putSecondPiece = new ConditionalCommand(new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
    new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm), ()->shouldStartWithCone);

    Eject eject = new Eject(gripper);

    addCommands(
        Commands.runOnce(()->gripper.setShouldGripCone(shouldStartWithCone),gripper),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
        new Eject(gripper),
            driveBack.alongWith(new GroundOnlyArms(gripper, lilArm, bigArm, telescopicArm)),
        new Ground(gripper, lilArm, bigArm, telescopicArm).alongWith(driveToCollect.asProxy()),//TODO: test otf traj and maybe change it
                Commands.runOnce(() -> {
          gripper.setShouldGripCone(false);
        }, gripper),
        returnTraj
            .alongWith(new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)),
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(
            areWeBlue ? new Translation2d(SwerveConstants.blueAligningX, 1.07) : new Translation2d(SwerveConstants.redAligningX, 1.07)), false),
        communityCheck.asProxy(),
        putSecondPiece,
        eject,
        new Ground(gripper, lilArm, bigArm, telescopicArm)
        );
  }
}
