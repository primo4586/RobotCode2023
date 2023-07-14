// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.LilArm;

public class IntakeSequential extends SequentialCommandGroup {
  public IntakeSequential(LilArm lilArm, BigArm bigArm) {
   //closes the solenoid
   MoveArmsToSetPointsBigFirst moveArmsToIntake = new MoveArmsToSetPointsBigFirst(bigArm, BigConstants.intakeSetPoint, lilArm, LilConstants.middleOfRobotSetPoint);
   addCommands(
    lilArm.closeLilArmSolenoid(),
    moveArmsToIntake
   );
  }
}
