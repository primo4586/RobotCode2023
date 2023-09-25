// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.commands.actions.gripper.Hold;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

public class MiddleOfBot extends SequentialCommandGroup {
  /** Creates a new MiddleOfBot. */
  public MiddleOfBot(LilArm lilArm, BigArm bigArm, TelescopicArm telescopicArm, Gripper gripper) {

    ParallelCommandGroup moveArms = new ParallelCommandGroup(
      bigArm.TurnBigArmToSetpoint(BigConstants.intakeSetPoint),
      lilArm.TurnLilArmToSetpoint(LilConstants.middleOfRobotSetPoint));
    
    addCommands(
        //hold,
      gripper.setSpeedCommand(gripper.shouldGripCone ? 0.1 : -0.1)
        .raceWith(telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint)),
      new Hold(gripper).raceWith(moveArms)
    );
  }
}
