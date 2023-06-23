package frc.robot.commands.actions;

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
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class CoolScoreDontOpen extends ParallelCommandGroup {
  public CoolScoreDontOpen(Swerve swerve,BigArm bigArm, LilArm lilArm, Gripper gripper , Objective objective) {
    
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

    Translation2d robotEndSpot;

    if (DriverStation.getAlliance() == Alliance.Blue)
      robotEndSpot = new Translation2d(SwerveConstants.blueAligningX,
          Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]));
    else
      robotEndSpot = new Translation2d(SwerveConstants.redAligningX,
          Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]));
    // Generate trajectory to alignment pose based on alliance color
    PathPlannerTrajectory trajectory = swerve.generateTrajectoryToAligmentPose(robotEndSpot);

    // Parallel command group to drive and score simultaneously
    ParallelCommandGroup driveAndArms = new ParallelCommandGroup(
        swerve.followTrajectory(trajectory, false).asProxy(),
        moveArms);

    boolean closenesCheck = Math.abs(swerve.getPose().getX() - robotEndSpot.getX()) < SwerveConstants.trajAccuracy &&
        Math.abs(swerve.getPose().getY() - robotEndSpot.getY()) < SwerveConstants.trajAccuracy;

    ConditionalCommand areWeThere = new ConditionalCommand(Commands.waitSeconds(0.2), driveAndArms,
        () -> closenesCheck);

    addCommands(
        driveAndArms,
        areWeThere);
  }
}
