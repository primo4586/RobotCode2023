package frc.robot.commands.actions;

import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO.Objective;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class CoolScore extends SequentialCommandGroup {
    public CoolScore(
            Swerve swerve,
            BigArm bigArm,
            LilArm lilArm,
            Gripper gripper,
            Objective objective,
            TelescopicArm telescopicArm) {

        ConditionalCommand moveArms = new ConditionalCommand(
            new PutItemInTheUpper(bigArm, lilArm, gripper, telescopicArm),
            new ConditionalCommand(
                new PutItemInTheMiddle(lilArm, bigArm, gripper, telescopicArm),
                new PutItemInTheLower(lilArm, bigArm, gripper, telescopicArm),
                () -> objective.nodeLevel == NodeLevel.MID),
            () -> objective.nodeLevel == NodeLevel.HYBRID
        );

        addCommands(
            new CoolScoreDriveSimple(swerve, objective).alongWith(moveArms).unless(()->!swerve.areWeCloseEnough()),
            new CoolScoreDriveSimple(swerve, objective),
            new CoolScoreDriveSimple(swerve, objective)
            
        );
    }
}
