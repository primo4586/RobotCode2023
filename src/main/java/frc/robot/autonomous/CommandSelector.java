package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

/** Add your docs here. */
public class CommandSelector {
    SendableChooser<Command> chooser;
    SelectCommand selector;
    Map<String,Command> commands;

    // creats new tab
    public CommandSelector(Map<String,Command> commands,String name){
        this(commands,Shuffleboard.getTab(name)); // calls the next constactor
    }

    // creats new arrayList and SendableChooser
    public CommandSelector(Map<String,Command> commands,ShuffleboardTab tab) {
        List<Entry<String, Command>> commandList = new ArrayList<>();
        this.commands = commands;
        chooser = new SendableChooser<>();
        
        // fills commandList with commands and their names
        for(Entry<String,Command> e : commands.entrySet()){
            commandList.add(e);
        }

        chooser.setDefaultOption(commandList.get(0).getKey(), commandList.get(0).getValue()); // command name + command itself
        for(int i = 1; i < commandList.size();i++){
            chooser.addOption(commandList.get(i).getKey(), commandList.get(i).getValue());
        }
        tab.add(chooser); // sending the chooser data to Shuffleboard
    }

    // starts the selected command
    public Command getCommand() {
        return chooser.getSelected();
    }
}
