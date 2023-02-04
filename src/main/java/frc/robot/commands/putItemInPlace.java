// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

public class putItemInPlace extends SequentialCommandGroup {
  /** Creates a new putItemInPlace. */
  public putItemInPlace(BigArm bigArm, LilArm lilArm, Gripper gripper) {
    MoveArmsToSetPoints firstSetPoint = new MoveArmsToSetPoints(bigArm, BigArmConstants.putItemInPlaceFirstSetPoint, lilArm, LilArmConstants.putItemInPlaceFirstSetPoint);
    MoveArmsToSetPoints secondSetPoint = new MoveArmsToSetPoints(bigArm, BigArmConstants.putItemInPlaceSecondSetPoint, lilArm, LilArmConstants.putItemInPlaceSecondSetPoint);
    ConditionalCommand closeSolenoid = new ConditionalCommand(lilArm.toggleLilArmSolenoid(), null, lilArm.isSolenoidOpen());
    addCommands(
      closeSolenoid,
      firstSetPoint,
      lilArm.toggleLilArmSolenoid(),
      //secondSetPoint,//check if this is nedded
      gripper.turnToSetPoint(GripperConstants.openGripperSetPoint)
    );
  }
}
