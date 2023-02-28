// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.autonomous.CommandSelector;
import frc.robot.commands.autoCommands.GamePieceThenCharge;
import frc.robot.commands.autoCommands.GamePieceThenDriveBack;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;
    private GamePieceThenCharge gamePieceThenCharge;
    private GamePieceThenDriveBack gamePieceThenDriveBackFarFromLoading;
    private GamePieceThenDriveBack gamePieceThenDriveBackNearLoading;

    public AutoContainer(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm){
        this.autoPaths = new HashMap<String, Command>();

        this.gamePieceThenCharge = new GamePieceThenCharge(gripper, bigArm, lilArm, swerve, true);
        this.gamePieceThenDriveBackFarFromLoading = new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, false);
        this.gamePieceThenDriveBackNearLoading = new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, true, false);

        autoPaths.put("No Auto", new InstantCommand());
        
        // autoPaths.put("near loading", gamePieceThenDriveBackNearLoading);
        // autoPaths.put("far from loadind", gamePieceThenDriveBackFarFromLoading);
        // autoPaths.put("charge", gamePieceThenCharge);
        autoPaths.put("Upper Cube", new GamePieceThenDriveBack(swerve, gripper, bigArm, lilArm, true, false, true));
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return this.autoSelector.getCommand();
      }
}
