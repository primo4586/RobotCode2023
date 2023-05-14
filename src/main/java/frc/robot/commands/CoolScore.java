package frc.robot.commands;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
  public CoolScore(Swerve swerve, BigArm bigArm, LilArm lilArm, Gripper gripper, Objective objective) {
    
    // Conditional command to move the arms based on the objective node level
    ConditionalCommand moveArms = new ConditionalCommand(
      new PutItemInTheUpper(bigArm, lilArm, gripper), // If node level is UPPER
      new ConditionalCommand(
        new PutItemInTheMiddle(lilArm, bigArm, gripper),
        Commands.none(), // TODO: replace with appropriate command if necessary
        () -> objective.nodeLevel == NodeLevel.MID // If node level is MID
      ),
      () -> objective.nodeLevel == NodeLevel.HYBRID // If node level is HYBRID
    );

    // Generate trajectory to alignment pose based on alliance color
    PathPlannerTrajectory blueTrajectory = swerve.generateTrajectoryToAligmentPose(
      new Translation2d(SwerveConstants.blueAligningX, Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]))
    );
    PathPlannerTrajectory redTrajectory = swerve.generateTrajectoryToAligmentPose(
      new Translation2d(SwerveConstants.redAligningX, Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]))
    );

    // Conditional command to follow the appropriate trajectory based on alliance color
    ConditionalCommand followTrajectory = new ConditionalCommand(
      swerve.followTrajectory(blueTrajectory, false), // Follow blue trajectory if alliance is Blue
      swerve.followTrajectory(redTrajectory, false), // Follow red trajectory if alliance is Red
      () -> DriverStation.getAlliance() == Alliance.Blue
    );

    // Parallel command group to drive and score simultaneously
    ParallelCommandGroup driveAndScore = new ParallelCommandGroup(followTrajectory, moveArms);

    // Add commands to the sequential command group
    addCommands(
      driveAndScore,
      Commands.waitSeconds(0.2),
      gripper.openGripper()
    );
  }
}

