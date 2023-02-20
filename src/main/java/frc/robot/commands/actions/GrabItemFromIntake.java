// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPoints;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

public class GrabItemFromIntake extends SequentialCommandGroup {
  public GrabItemFromIntake(LilArm lilArm, BigArm bigArm, Gripper gripper) {
   //closes the solenoid
   ConditionalCommand closeSolenoid = new ConditionalCommand(lilArm.toggleLilArmSolenoid(), Commands.none(), lilArm::isSolenoidOpen);

   MoveArmsToSetPoints moveArmsToIntake = new MoveArmsToSetPoints(bigArm, BigArmConstants.intakeSetPoint, lilArm, LilArmConstants.intakeSetPoint);
   MoveArmsToSetPoints moveArmsToMiddleOfBot = new MoveArmsToSetPoints(bigArm, BigArmConstants.middleOfRobotSetPoint, lilArm, LilArmConstants.middleOfRobotSetPoint);

   ConditionalCommand openGripper = new ConditionalCommand(gripper.ToggleGripper(), Commands.none(),()-> gripper.isGripperOpen());

   addCommands(
     closeSolenoid,
     openGripper,
     moveArmsToIntake,
     gripper.ToggleGripper(),
     moveArmsToMiddleOfBot
   );
  }
}
