package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

public class Ground extends SequentialCommandGroup {

  public Ground(Gripper gripper, LilArm lilArm, BigArm bigArm, TelescopicArm telescopicArm) {

    ParallelCommandGroup moveArms = new ParallelCommandGroup(bigArm.TurnBigArmToSetpoint(BigConstants.groundSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.groundSetPoint));

    addCommands(
      telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint),
      moveArms,
      gripper.getCollectCommand(),
      telescopicArm.putTelesInSetpoint(TelescopicArmConstants.groundSetPoint)
    );
  }
}
