// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.LilArm;

public class IntakeSequential extends SequentialCommandGroup {
  public IntakeSequential(LilArm lilArm, BigArm bigArm) {
   //closes the solenoid
   MoveArmsToSetPointsBigFirst moveArmsToIntake = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.intakeSetPoint, lilArm, LilArmConstants.intakeSetPoint);
   MoveArmsToSetPointsBigFirst moveArmsToIntake2 = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.intakeSetPoint, lilArm, LilArmConstants.intakeSetPoint);
  
  
   

   addCommands(
    // gripper.closeGripper(),
    lilArm.closeLilArmSolenoid(),
    /*bigArm.TurnBigArmToSetpoint(BigArmConstants.cubeUpperFirstSetPoint).unless(()->bigArm.getCurrentArmPosition()<28000),
    lilArm.TurnLilArmToSetpoint(LilArmConstants.intakeSetPoint).unless(() -> lilArm.getCurrentArmPosition() > LilArmConstants.intakeSetPoint),//||bigArm.getCurrentArmAngle()>BigArmConstants.intakeReturnDeadZone),
    */
    moveArmsToIntake
     //gripper.openGripper()
     //gripper.closeGripper(
     //moveArmsToMiddleOfBot
   );
  }
}
