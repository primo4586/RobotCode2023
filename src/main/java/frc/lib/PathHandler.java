package frc.lib;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

/**
 * Handles loading paths from PathWeaver export json files to Trajectories
 * @see {https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/creating-following-trajectory.html}
 */
public class PathHandler {
    public static PathHandler instance;

    public static PathHandler getInstance(){
        if(instance == null){
            instance = new PathHandler();
        }
        return instance;
    }

    /**
     * Loads the given PathWeaver path from the JSON export file as a trajectory
     * @param path Directory to the path's location (always will be under src/main/deploy)
     * @return The path's Trajectory
     */
    public Trajectory loadPath(String path){
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
        return trajectory;
    }
}
