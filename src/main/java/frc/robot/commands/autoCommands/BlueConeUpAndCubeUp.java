// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;


import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.IntakeParallel;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueConeUpAndCubeUp extends SequentialCommandGroup {
  /** Creates a new BlueConeUpAndCubeMid. */
  public BlueConeUpAndCubeUp(BigArm bigArm, LilArm lilArm, Gripper gripper, Swerve swerve) {
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);
    
    BlueDriveBackAndGround driveBackAndGround = new BlueDriveBackAndGround(swerve, gripper, bigArm, lilArm, false);
    addCommands(
      Commands.runOnce(() -> {
        gripper.setShouldGripCone(true);
        gripper.turnOnLed();
      }, gripper),
      putItemInTheUpper,
      Commands.waitSeconds(0.4),
      gripper.openGripper(),
      Commands.waitSeconds(0.3),
      lilArm.closeLilArmSolenoid(),
      driveBackAndGround,
      Commands.waitSeconds(0.2),
      gripper.closeGripper(),
      Commands.waitSeconds(0.2),
      swerve.followTrajectory(PathPlanner.loadPath("blueUpperCubeReturn", AutoConstants.pathConstraints), false)
      .alongWith(new IntakeParallel(lilArm, bigArm)),
      Commands.runOnce(() -> {
        gripper.setShouldGripCone(false);
        gripper.turnOnLed();
      }, gripper),
      new PutItemInTheUpper(bigArm, lilArm, gripper),
      lilArm.openLilArmSolenoid(),
      Commands.waitSeconds(0.4),
      gripper.openGripper(),
      Commands.waitSeconds(0.4),
      new IntakeSequential(lilArm, bigArm).alongWith(Commands.waitSeconds(0.2).andThen(gripper.closeGripper()))
    );
  }
}
