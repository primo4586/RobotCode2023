package frc.robot.commands.actions;


import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class CoolScore extends SequentialCommandGroup {
  public CoolScore(Swerve swerve, BigArm bigArm, LilArm lilArm, Gripper gripper, Objective objective, TelescopicArm telescopicArm) {
    
    // Conditional command to move the arms based on the objective node level
    ConditionalCommand moveArms = new ConditionalCommand(
      new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm), // If node level is UPPER
      new ConditionalCommand(
        new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
        Commands.none(),
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

    // Parallel command group to drive and score simultaneously
    ParallelCommandGroup driveAndArms = new ParallelCommandGroup(
        swerve.followTrajectory(swerve.generateTrajectoryToAligmentPose(robotEndSpot), false).asProxy(),
        moveArms);

    BooleanSupplier closenesCheck = ()->Math.abs(swerve.getPose().getX() - robotEndSpot.getX()) < SwerveConstants.trajAccuracy &&
        Math.abs(swerve.getPose().getY() - robotEndSpot.getY()) < SwerveConstants.trajAccuracy;

    ConditionalCommand areWeThere = new ConditionalCommand(Commands.waitSeconds(0.2), driveAndArms, closenesCheck);

    addCommands(
      driveAndArms,
      areWeThere
    );
  }
}
