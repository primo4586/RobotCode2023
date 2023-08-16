// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.Constants.TelescopicArmConstants;

public class PutItemInTheMiddle extends SequentialCommandGroup {
  public PutItemInTheMiddle(LilArm lilArm, BigArm bigArm, Gripper gripper, TelescopicArm telescopicArm) {
    //cone and cube setPoints
    ParallelCommandGroup coneMiddleSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.coneMiddleSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.coneMiddleSetPoint),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneMiddleSetPoint));

    ParallelCommandGroup cubeMiddleSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.cubeMiddleSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.cubeMiddleSetPoint),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneMiddleSetPoint));

    //check if we put cone or cube
    ConditionalCommand putArmsInMiddleSetPoint = new ConditionalCommand(coneMiddleSetPoint, cubeMiddleSetPoint,gripper::getShouldGripCone);

    addCommands(
      putArmsInMiddleSetPoint
    );
  }
}
