// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

public class PutItemInTheUpper extends SequentialCommandGroup {
  /** Creates a new putItemInPlace. */
  public PutItemInTheUpper(BigArm bigArm, LilArm lilArm, Gripper gripper) {
    
    //closes the solenoid
    ConditionalCommand closeSolenoid = new ConditionalCommand(lilArm.toggleLilArmSolenoid(), Commands.none(), lilArm::isSolenoidOpen);

    //cone and cube setPoints
    MoveArmsToSetPoints coneUpperSetPoint = new MoveArmsToSetPoints(bigArm, BigArmConstants.coneUpperSetPoint, lilArm, LilArmConstants.coneUpperSetPoint);
    MoveArmsToSetPoints cubeUpperSetPoint = new MoveArmsToSetPoints(bigArm, BigArmConstants.cubeUpperSetPoint, lilArm, LilArmConstants.cubeUpperSetPoint);

    //check if we put cone or cube
    ConditionalCommand putArmsInUpperSetPoint = new ConditionalCommand(coneUpperSetPoint, cubeUpperSetPoint, gripper::getShouldGripCone);

    addCommands(
      closeSolenoid,
      putArmsInUpperSetPoint,
      lilArm.toggleLilArmSolenoid(),
      gripper.turnToSetPoint(GripperConstants.openGripperSetPoint)
    );
  }
}
