// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirat;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

public class PutItemInTheUpper extends SequentialCommandGroup {
  /** Creates a new putItemInPlace.  */
  public PutItemInTheUpper(BigArm bigArm, LilArm lilArm, Gripper gripper) {
    //cone and cube setPoints
    //MoveArmsToSetPointsLilFirst coneUpperSetPoint = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.coneUpperSetPoint, lilArm, LilArmConstants.coneUpperSetPoint);
    MoveArmsToSetPointsBigFirat cubeUpperFirstSetPoint = new MoveArmsToSetPointsBigFirat(bigArm, BigArmConstants.cubeUpperFirstSetPoint, lilArm, LilArmConstants.cubeUpperFirstSetPoint);
    MoveArmsToSetPointsBigFirat cubeUpperSecondSetPoint = new MoveArmsToSetPointsBigFirat(bigArm, BigArmConstants.cubeUpperSecondSetPoint, lilArm, LilArmConstants.cubeUpperFirstSetPoint);
    MoveArmsToSetPointsLilFirst cubeUpperFinalSetPoint = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.cubeUpperFinalSetPoint, lilArm, LilArmConstants.cubeUpperFinalSetPoint);

    
    MoveArmsToSetPointsLilFirst coneUpperFinalSetPoint = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.coneUpperFinalSetPoint, lilArm, LilArmConstants.coneUpperFinalSetPoint);
    //check if we put cone or cube
    ConditionalCommand finalSetPoint = new ConditionalCommand(coneUpperFinalSetPoint, cubeUpperFinalSetPoint, gripper::getShouldGripCone);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      cubeUpperFirstSetPoint,
      cubeUpperSecondSetPoint,
      finalSetPoint,
      lilArm.openLilArmSolenoid()
    );
  }
}
