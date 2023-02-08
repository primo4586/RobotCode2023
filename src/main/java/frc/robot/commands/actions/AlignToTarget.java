package frc.robot.commands.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class AlignToTarget extends SequentialCommandGroup {

  
  public AlignToTarget(Swerve swerve, Gripper gripper) {
    double AligningY;
    int closestXId = swerve.whereToAlign(gripper);
    Rotation2d aligningRotation = new Rotation2d(swerve.getYaw().getDegrees());
    Pose2d alignLocation;

    if(DriverStation.getAlliance()==DriverStation.Alliance.Red)
      AligningY = SwerveConstants.redAligningY;
    else
      AligningY = SwerveConstants.blueAligningY;

    if(gripper.getShouldGripCone())
      alignLocation = new Pose2d(SwerveConstants.coneAligningX[closestXId], AligningY, aligningRotation);
    else
      alignLocation = new Pose2d(SwerveConstants.cubeAligningX[closestXId], AligningY, aligningRotation);
    addCommands(
    
    );
  }
}
