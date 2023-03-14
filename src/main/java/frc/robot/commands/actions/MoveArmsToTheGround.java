package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
public class MoveArmsToTheGround extends SequentialCommandGroup {
  public MoveArmsToTheGround(Gripper gripper, LilArm lilArm, BigArm bigArm) {

    IntakeSequential grabItemFromIntake = new IntakeSequential(lilArm, bigArm);
    MoveArmsToSetPointsLilFirst moveArmsToGround = new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.groundSetPoint);
  
    addCommands(
      gripper.closeGripper(),
      grabItemFromIntake,
      moveArmsToGround,
      lilArm.openLilArmSolenoid()
    );
  }
}
