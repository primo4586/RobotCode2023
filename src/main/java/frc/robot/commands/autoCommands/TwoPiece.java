// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.commands.actions.AutoUpper;
import frc.robot.commands.actions.Ground;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class TwoPiece extends SequentialCommandGroup {
  public TwoPiece(boolean shouldStartWithCone, boolean shouldFinishWithCone, boolean areWeBlue, BigArm bigArm,
      LilArm lilArm, Gripper gripper, Swerve swerve, TelescopicArm telescopicArm) {

    ConditionalCommand putSecondPiece = new ConditionalCommand(
        new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
        new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
        () -> shouldStartWithCone == shouldFinishWithCone);

    ConditionalCommand returnTraj = new ConditionalCommand(
        swerve
            .followTrajectory(PathPlanner.loadPath(shouldFinishWithCone ? "blueUpperConeReturn" : "blueUpperCubeReturn",
                AutoConstants.pathConstraints), false),
        swerve.followTrajectory(PathPlanner.loadPath(shouldFinishWithCone ? "redUpperConeReturn" : "redUpperCubeReturn",
            AutoConstants.pathConstraints), false),
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

    Eject eject = new Eject(gripper);

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
        //collectCheck,
        returnTraj.alongWith(new AutoUpper(bigArm, lilArm, gripper, telescopicArm)),
        communityCheck,
        putSecondPiece,
        eject,
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint),
        bigArm.TurnBigArmToSetpoint(BigConstants.middleOfRobotSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.middleOfRobotSetPoint));
  }
}
