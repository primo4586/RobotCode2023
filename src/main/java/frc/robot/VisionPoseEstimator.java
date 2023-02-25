package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

// more descriptive comments
public class VisionPoseEstimator {

    private AprilTagFieldLayout apriltagLayout;
    private NetworkTable limelightNT;

    public VisionPoseEstimator() {
        try {
            // Loads up the field layout - Essentially a hashmap of every AprilTag's ID and
            // its' position on the field.
            apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            // NT Table for limelight values
            limelightNT = NetworkTableInstance.getDefault().getTable("limelight");

        } catch (IOException e) {
            apriltagLayout = null;
            DriverStation.reportError("AprilTag Field was not able to load!!! Vision data is not able to be processed.",
                    true);
            e.printStackTrace();
        }
    }

    /**
     * Gets the last estimated position from vision
     * 
     * @param prevEstimatedRobotPose The last estimated position to use as a
     *                               reference. (Only relevant when using pose
     *                               strategies relevant to a reference pose)
     * @return The new estimated position using the latest vision
     */
    public Pair<Pose3d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (apriltagLayout == null) {
            DriverStation.reportError("AprilTagField was not able to load!! Cannot provide position!!", false);
            return null;
        }

        
        var resultsArray = limelightNT.getEntry("botpose").getDoubleArray(new double[7]);

        // Rejects the result only if the result doesn't exist or if the ambguity is
        // over 20% and therefore the data is unreliable.
        var hasTargets = limelightNT.getEntry("tv").getDouble(0) == 1;
        if (hasTargets) {

            
            var estimatedPose = new Pose3d(resultsArray[0], resultsArray[1], resultsArray[2], new Rotation3d(resultsArray[3], resultsArray[4], resultsArray[5]));
            var latencyInSeconds = resultsArray[6] / 1000.0;
            
            var timestamp = Timer.getFPGATimestamp() - latencyInSeconds;
    
            // Returns the data with the Pose3d, and the timestamp relative to the robot's time of when the pose was estimated at. (EstimatedRobotPose)
            return Pair.of(estimatedPose, timestamp);
        } else {
            return null;
        }
    }

    public AprilTagFieldLayout getApriltagLayout() {
        return apriltagLayout;
    }

}
