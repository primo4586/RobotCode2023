// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

public class PutItemInTheMiddle extends SequentialCommandGroup {
  public PutItemInTheMiddle(LilArm lilArm, BigArm bigArm, Gripper gripper) {
    //cone and cube setPoints
    MoveArmsToSetPointsLilFirst coneMiddleSetPoint = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.coneMiddleSetPoint, lilArm, LilArmConstants.coneMiddleSetPoint);
    MoveArmsToSetPointsLilFirst cubeMiddleSetPoint = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.cubeMiddleSetPoint, lilArm, LilArmConstants.cubeMiddleSetPoint);

    //check if we put cone or cube
    ConditionalCommand putArmsInMiddleSetPoint = new ConditionalCommand(coneMiddleSetPoint, cubeMiddleSetPoint,gripper::getShouldGripCone);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      putArmsInMiddleSetPoint
    );
  }
}
