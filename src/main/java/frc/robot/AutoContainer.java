// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.autonomous.CommandSelector;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.autoCommands.ChargeAlignOtherSide;
import frc.robot.commands.autoCommands.GamePieceThenDriveBack;
import frc.robot.commands.autoCommands.RedConeUpAndCubeUp;
import frc.robot.commands.autoCommands.RedCubeUpAndMidd;
import frc.robot.commands.usefulAutos.BlueConeUpAndCubeUp;
import frc.robot.commands.usefulAutos.BlueCubeUpAndMidd;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;

    public AutoContainer(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm){
        this.autoPaths = new HashMap<String, Command>(); 

        //TwoPieces twoPieces = new TwoPieces(swerve, gripper, bigArm, lilArm, true, true, false);
        Command cubeUpper = Commands.runOnce(() -> gripper.setShouldGripCone(false), gripper).andThen(new PutItemInTheUpper(bigArm, lilArm, gripper)).andThen(Commands.waitSeconds(0.3)).andThen(gripper.openGripper());
        Command cubeUpper2 = Commands.runOnce(() -> gripper.setShouldGripCone(false), gripper).andThen(new PutItemInTheUpper(bigArm, lilArm, gripper)).andThen(Commands.waitSeconds(0.3)).andThen(gripper.openGripper());

        autoPaths.put("No Auto", new InstantCommand());
        autoPaths.put("blue two cubes", new BlueCubeUpAndMidd(swerve, gripper, bigArm, lilArm));
        autoPaths.put("red two cubes", new RedCubeUpAndMidd(swerve, gripper, bigArm, lilArm, true, false, false));
        autoPaths.put("blue cone and cube", new BlueConeUpAndCubeUp(
            bigArm, lilArm, gripper, swerve));
        autoPaths.put("red cone and cube", new RedConeUpAndCubeUp(bigArm, lilArm, gripper, swerve));
        //test charge station before reload charge
        //autoPaths.put("reload charge", new reloadCharge(swerve, gripper, bigArm, lilArm, true, false, false));
        //autoPaths.put("Charge Station", new ChargeAlign(swerve));

        autoPaths.put("Cube Charge", cubeUpper.andThen(Commands.waitSeconds(0.5)).andThen(new ParallelCommandGroup(new IntakeSequential(lilArm, bigArm),Commands.waitSeconds(0.7).andThen(new ChargeAlignOtherSide(swerve)))));
        autoPaths.put("super charge", new ChargeAlignOtherSide(swerve));
        autoPaths.put("Cone Upper FULL", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, true));
        autoPaths.put("Cube Upper FULL", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, false));
        autoPaths.put("Cube MIDDLE FULL", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, false, false, false));
        autoPaths.put("CONE MIDDLE FULL", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, false, false, true));
        autoPaths.put("Drive By Time", swerve.driveForTimeAtSpeed(new Translation2d(-2, 0), 3));
        autoPaths.put("Cube Upper Arm", cubeUpper2);
        autoPaths.put("Cone Upper Arm", Commands.runOnce(() -> gripper.setShouldGripCone(true), gripper).andThen(new PutItemInTheUpper(bigArm, lilArm, gripper)).andThen(Commands.waitSeconds(0.3)).andThen(gripper.openGripper()));
        autoPaths.put("Cone middle Arm", Commands.runOnce(() -> gripper.setShouldGripCone(true), gripper).andThen(new PutItemInTheMiddle(lilArm, bigArm, gripper)).andThen(Commands.waitSeconds(0.3)).andThen(gripper.openGripper()));
        autoPaths.put("Cube middle Arm", Commands.runOnce(() -> gripper.setShouldGripCone(false), gripper).andThen(new PutItemInTheMiddle(lilArm, bigArm, gripper)).andThen(Commands.waitSeconds(0.3)).andThen(gripper.openGripper()));
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
      }
}
    