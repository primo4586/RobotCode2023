package frc.robot.commands.actions;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;

import com.pathplanner.lib.PathPoint;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class CoolScore extends SequentialCommandGroup {
    public CoolScore(
            Swerve swerve, BigArm bigArm, LilArm lilArm, Gripper gripper,
            Objective objective, TelescopicArm telescopicArm) {

        Alliance alliance = DriverStation.getAlliance();
        double aligningX = (alliance == Alliance.Blue) ? SwerveConstants.blueAligningX : SwerveConstants.redAligningX;
        double aligningY = Units.inchesToMeters(SwerveConstants.redAligningYAxis[objective.getNodeRow()]);
        Translation2d endSpotTranslatio2d = new Translation2d(aligningX, aligningY);
        PathPoint endPoint = new PathPoint(endSpotTranslatio2d, new Rotation2d(0));

        Pose2d pose = swerve.getPose();
        PathPoint robotPose = new PathPoint(
                pose.getTranslation(), swerve.getYaw(), swerve.getStates()[0].speedMetersPerSecond);

        ConditionalCommand moveArms = new ConditionalCommand(
                new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
                new ConditionalCommand(
                        new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
                        Commands.none(),
                        () -> objective.nodeLevel == NodeLevel.MID),
                () -> objective.nodeLevel == NodeLevel.HYBRID);

        Command drive;
        CommandBase lowerDrive = Commands.none();

        if (pose.getY() > 3.35) {
            if (pose.getX() > 5.4) {
                PathPoint alignToEnterCommunity = new PathPoint(new Translation2d(5.42, 4.7), new Rotation2d(0));
                PathPoint enterCommunity = new PathPoint(new Translation2d(2.3, 4.7), new Rotation2d(0));
                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(
                                List.of(robotPose, alignToEnterCommunity, enterCommunity, endPoint)),
                        false);
            } else if (pose.getX() > 2.47) {
                PathPoint enterCommunity = new PathPoint(new Translation2d(2.3, 4.7), new Rotation2d(0));
                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(List.of(robotPose, enterCommunity, endPoint)),
                        false);
            } else {
                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(List.of(robotPose, endPoint)),
                        false);
            }
        } else {
            if (pose.getX() > 5.4) {
                PathPoint alignToEnterCommunity = new PathPoint(new Translation2d(5.42, 0.9), new Rotation2d(0));
                PathPoint enterCommunity = new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0));
                lowerDrive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(
                                List.of(robotPose, alignToEnterCommunity, enterCommunity)),
                        false).asProxy();

                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(
                                List.of(new PathPoint(pose.getTranslation(), swerve.getYaw(),
                                        swerve.getStates()[0].speedMetersPerSecond), endPoint)),
                        false).asProxy();
            } else if (pose.getX() > 3.3) {
                PathPoint enterCommunity = new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0));
                lowerDrive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(List.of(robotPose, enterCommunity)),
                        false).asProxy();

                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(
                                List.of(new PathPoint(pose.getTranslation(), swerve.getYaw(),
                                        swerve.getStates()[0].speedMetersPerSecond), endPoint)),
                        false).asProxy();

            } else if (pose.getX() > 2.47) {
                PathPoint enterCommunity = new PathPoint(new Translation2d(2.3, 0.9), new Rotation2d(0));
                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(List.of(robotPose, enterCommunity, endPoint)),
                        false);
            } else {
                drive = swerve.followTrajectory(
                        swerve.generateTrajectoryToAligmentPose(List.of(robotPose, endPoint)),
                        false);
            }
        }

        BooleanSupplier closenesCheck = () -> Math
                .abs(swerve.getPose().getX() - endSpotTranslatio2d.getX()) < SwerveConstants.trajAccuracy &&
                Math.abs(swerve.getPose().getY() - endSpotTranslatio2d.getY()) < SwerveConstants.trajAccuracy;

        ConditionalCommand areWeThere = new ConditionalCommand(Commands.none(), swerve.followTrajectory(
                swerve.generateTrajectoryToAligmentPose(List.of(new PathPoint(pose.getTranslation(), swerve.getYaw(),
                        swerve.getStates()[0].speedMetersPerSecond), endPoint)),
                false).asProxy(), closenesCheck);

        addCommands(
                lowerDrive,
                drive.alongWith(moveArms),
                areWeThere);
    }
}
