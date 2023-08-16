// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

public class PutItemInTheUpper extends SequentialCommandGroup {
  /** Creates a new putItemInPlace.  */
  public PutItemInTheUpper(BigArm bigArm, LilArm lilArm, Gripper gripper,TelescopicArm telescopicArm) {
    //cone and cube setPoints
    ParallelCommandGroup coneUpperSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.coneUpperSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.coneUpperSetPoint),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneUpperSetPoint));

    ParallelCommandGroup cubeUpperSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.cubeUpperSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.cubeUpperSetPoint),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneUpperSetPoint));

    // check if we put cone or cube
    ConditionalCommand putArmsInUpperSetPoint = new ConditionalCommand(coneUpperSetPoint, cubeUpperSetPoint,
        gripper::getShouldGripCone);

    addCommands(
        putArmsInUpperSetPoint);
  }
}
