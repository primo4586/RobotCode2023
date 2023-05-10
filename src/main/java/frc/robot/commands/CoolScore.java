package frc.robot.commands;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
public class CoolScore extends SequentialCommandGroup {
  public CoolScore(Swerve swerve,BigArm bigArm, LilArm lilArm, Gripper gripper , Objective objective) {
    
    ConditionalCommand moveArms = new ConditionalCommand(new PutItemInTheUpper(bigArm, lilArm, gripper),
     new ConditionalCommand(new PutItemInTheMiddle(lilArm, bigArm, gripper), Commands.none(), ()->objective.nodeLevel == NodeLevel.MID),//TODO: replace commands.none
     ()-> objective.nodeLevel==NodeLevel.HYBRID);

     PathPlannerTrajectory blueTrajectory = swerve.generateTrajectoryToAligmentPose(new Translation2d(SwerveConstants.blueAligningX,SwerveConstants.aligningYAxis[objective.getNodeRow()]));
     PathPlannerTrajectory redTrajectory = swerve.generateTrajectoryToAligmentPose(new Translation2d(SwerveConstants.redAligningX,SwerveConstants.aligningYAxis[objective.getNodeRow()]));

     ConditionalCommand folowTrajecctory = new ConditionalCommand(
      swerve.followTrajectory(blueTrajectory, false),
      swerve.followTrajectory(redTrajectory, false),()-> DriverStation.getAlliance()==Alliance.Blue);

    ParallelCommandGroup driveAndScore = new ParallelCommandGroup(folowTrajecctory, moveArms);

    addCommands(
      driveAndScore,
      Commands.waitSeconds(0.2),
      gripper.openGripper()
    );
  }
}
