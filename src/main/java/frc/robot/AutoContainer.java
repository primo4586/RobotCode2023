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
import frc.lib.util.PrimoShuffleboard;
import frc.robot.autonomous.CommandSelector;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.autoCommands.ChargeAlign;
import frc.robot.commands.autoCommands.ChargeAlignOtherSide;
import frc.robot.commands.autoCommands.GamePieceThenDriveBack;
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


        autoPaths.put("No Auto", new InstantCommand());
        // autoPaths.put("Cube Move Arm By Time", lilArm.speedByTime(0.3, 1.5));
        // autoPaths.put("Cube Timed", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, false));
        autoPaths.put("Cone Upper FULL", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, true));
        autoPaths.put("Cube Upper FULL", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, false));
        autoPaths.put("Drive By Time", swerve.driveForTimeAtSpeed(new Translation2d(1.25, 0), 2.5));
        autoPaths.put("Drive By Time AND CUBE", lilArm.speedByTime(0.6, 0.75).andThen(swerve.driveForTimeAtSpeed(new Translation2d(1.25, 0), 2.5)));
        autoPaths.put("super charge", new ChargeAlignOtherSide(swerve));
        autoPaths.put("Charge Station", new ChargeAlign(swerve));
        

        Command driveAndChargeAngle = swerve.driveForTimeAtSpeed(new Translation2d(-1.75, 0), 3);//.andThen(swerve.chargeStationAlign());

        autoPaths.put("Drive And Charge Angle", driveAndChargeAngle);
        autoPaths.put("Cube Upper Arm", Commands.runOnce(() -> gripper.setShouldGripCone(false), gripper).andThen(new PutItemInTheUpper(bigArm, lilArm, gripper).andThen(gripper.openGripper())));
        autoPaths.put("Cube Lower Arm", Commands.runOnce(() -> gripper.setShouldGripCone(false), gripper).andThen(new PutItemInTheMiddle(lilArm, bigArm, gripper).andThen(gripper.openGripper())));
        autoPaths.put("Cone Upper Arm", Commands.runOnce(() -> gripper.setShouldGripCone(true), gripper).andThen(new PutItemInTheUpper(bigArm, lilArm, gripper).andThen(gripper.openGripper())));
        autoPaths.put("Cone Lower Arm", Commands.runOnce(() -> gripper.setShouldGripCone(true), gripper).andThen(new PutItemInTheMiddle(lilArm, bigArm, gripper)).andThen(gripper.openGripper()));
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
      }
}
    