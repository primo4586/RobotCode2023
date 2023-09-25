// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.MiddleOfBot;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.gripper.Eject;
import frc.robot.commands.actions.gripper.Hold;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
public class PieceCharge extends SequentialCommandGroup {
  public PieceCharge(Boolean shouldPutCone, Boolean shouldDriveAwayFromDriver, Swerve swerve, LilArm lilArm,
      BigArm bigArm, TelescopicArm telescopicArm, Gripper gripper) {
    
    ConditionalCommand startClimb = new ConditionalCommand(swerve.driveAtSpeed(-2.0), swerve.driveAtSpeed(2.0),
      ()-> shouldDriveAwayFromDriver);

    addCommands(
    gripper.setShouldGripConeCommand(shouldPutCone),
    new Hold(gripper).raceWith(new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm)),
    new Eject(gripper),
    startClimb.until(()->Math.abs(swerve.getRoll())>9.5).alongWith(new MiddleOfBot(lilArm, bigArm, telescopicArm, gripper)),
    new PIDBalance(swerve),
    new PIDBalance(swerve)
    );
  }
}
