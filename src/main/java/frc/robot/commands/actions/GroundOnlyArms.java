package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

public class GroundOnlyArms extends ParallelCommandGroup {

  public GroundOnlyArms(LilArm lilArm, BigArm bigArm, TelescopicArm telescopicArm) {
    addCommands(
      telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint),
      bigArm.TurnBigArmToSetpoint(BigConstants.groundSetPoint),
      lilArm.TurnLilArmToSetpoint(LilConstants.groundSetPoint)
    );
  }
}
