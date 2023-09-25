// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.commands.actions.gripper.Hold;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoUpper extends SequentialCommandGroup {
  /** Creates a new AutoUpper. */
  public AutoUpper(BigArm bigArm, LilArm lilArm, Gripper gripper,TelescopicArm telescopicArm) {
    //cone and cube setPoints
    ParallelCommandGroup coneUpperSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.coneUpperSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.coneUpperSetPoint));

    ParallelCommandGroup cubeUpperSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.cubeUpperSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.cubeUpperSetPoint));

    // check if we put cone or cube
    ConditionalCommand putArmsInUpperSetPoint = new ConditionalCommand(coneUpperSetPoint, cubeUpperSetPoint,
        gripper::getShouldGripCone);

    addCommands(
        //hold,
        gripper.setSpeedCommand(gripper.shouldGripCone?0.1:-0.1).alongWith(telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint)),
        new Hold(gripper).raceWith(putArmsInUpperSetPoint));
  }
}
