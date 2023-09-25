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
import frc.robot.commands.actions.gripper.Hold;

public class PutItemInTheMiddle extends SequentialCommandGroup {
  public PutItemInTheMiddle(LilArm lilArm, BigArm bigArm, Gripper gripper, TelescopicArm telescopicArm) {
    //cone and cube setPoints
    ParallelCommandGroup coneMiddleSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.coneMiddleSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.coneMiddleSetPoint));

    ParallelCommandGroup cubeMiddleSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.cubeMiddleSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.cubeMiddleSetPoint));

    //check if we put cone or cube
    ConditionalCommand putArmsInMiddleSetPoint = new ConditionalCommand(coneMiddleSetPoint, cubeMiddleSetPoint,gripper::getShouldGripCone);
    ConditionalCommand putTelesInsetPoint = new ConditionalCommand(
      telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneMiddleSetPoint), 
      telescopicArm.putTelesInSetpoint(TelescopicArmConstants.cubeMiddleSetPoint), gripper::getShouldGripCone);

    Hold hold = new Hold(gripper);

    addCommands(
      //hold,
      gripper.setSpeedCommand(gripper.shouldGripCone ? 0.1 : -0.1)
        .raceWith(telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint)),
      hold.raceWith(putArmsInMiddleSetPoint),
      putTelesInsetPoint
    );
  }
}
