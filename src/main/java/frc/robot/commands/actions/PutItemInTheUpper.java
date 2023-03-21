// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

public class PutItemInTheUpper extends SequentialCommandGroup {
  /** Creates a new putItemInPlace.  */
  public PutItemInTheUpper(BigArm bigArm, LilArm lilArm, Gripper gripper) {
    //cone and cube setPoints
    //MoveArmsToSetPointsLilFirst coneUpperSetPoint = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.coneUpperSetPoint, lilArm, LilArmConstants.coneUpperSetPoint);
    //MoveArmsToSetPointsBigFirst cubeUpperFirstSetPoint = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.cubeUpperFirstSetPoint, lilArm, LilArmConstants.cubeUpperFirstSetPoint);
    //MoveArmsToSetPointsBigFirst cubeUpperSecondSetPoint = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.cubeUpperSecondSetPoint, lilArm, LilArmConstants.cubeUpperFirstSetPoint);
    MoveArmsParallel cubeUpperFinalSetPoint = new MoveArmsParallel(bigArm, BigArmConstants.cubeUpperFinalSetPoint, lilArm, LilArmConstants.cubeUpperFinalSetPoint);

    
    MoveArmsParallel coneUpperFinalSetPoint = new MoveArmsParallel(bigArm, BigArmConstants.coneUpperFinalSetPoint, lilArm, LilArmConstants.coneUpperFinalSetPoint);
    //Command coneUpper = new ParallelCommandGroup(lilArm.TurnLilArmToSetpointWithCustomPID(LilArmConstants.coneUpperFinalSetPoint, LilArmConstants.lilArmUpperConePID),bigArm.TurnBigArmToSetpoint(BigArmConstants.coneUpperFinalSetPoint));
    Command coneUpper = new ParallelCommandGroup(lilArm.TurnLilArmToSetpoint(LilArmConstants.coneUpperFinalSetPoint),bigArm.TurnBigArmToSetpoint(BigArmConstants.coneUpperFinalSetPoint));

    //check if we put cone or cube
    ConditionalCommand finalSetPoint = new ConditionalCommand(coneUpper, cubeUpperFinalSetPoint, gripper::getShouldGripCone);

    // -54806 BIG
    // -1742 LIL
    ConditionalCommand middleCone = new ConditionalCommand(lilArm.TurnLilArmToSetpoint(LilArmConstants.coneMiddleSetPoint), Commands.none(), gripper::getShouldGripCone);
    addCommands(
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      //cubeUpperFirstSetPoint,
      //cubeUpperSecondSetPoint,
      // middleCone,
      finalSetPoint,
      lilArm.openLilArmSolenoid()
    );
  }
}
