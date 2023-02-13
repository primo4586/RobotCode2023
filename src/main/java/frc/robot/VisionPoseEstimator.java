package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

// more descriptive comments
public class VisionPoseEstimator {

    public PhotonCamera limeLight;
    public PhotonPoseEstimator robotPoseEstimator;
    private AprilTagFieldLayout apriltagLayout;

    public VisionPoseEstimator(PoseStrategy strategy) {
        try {
            // Loads up the field layout - Essentially a hashmap of every AprilTag's ID and
            // its' position on the field.
            apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            // Photon wrapper for the NT data.
            limeLight = new PhotonCamera(VisionConstants.cameraName);

            var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
            camList.add(new Pair<PhotonCamera, Transform3d>(limeLight, VisionConstants.robotToCam));

            // Uses the given pose strategy to take the data from the vision camera and
            // according to the strategy, gives the estimated position from vision data.
            robotPoseEstimator = new PhotonPoseEstimator(apriltagLayout, strategy, limeLight, VisionConstants.robotToCam);
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
    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (apriltagLayout == null) {
            DriverStation.reportError("AprilTagField was not able to load!! Cannot provide position!!", false);
            return null;
        }

        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        // Gets the latest best target result from the pose estimator, using the
        // PoseStrategy it's set to.
        // Returns a pair of the estimated position, and the latency it took to generate
        // it.
        Optional<EstimatedRobotPose> result = robotPoseEstimator.update();

        // Additional of the latest result, so we can have the pose ambguity data
        PhotonPipelineResult lastResult = limeLight.getLatestResult();

        // Rejects the result only if the result doesn't exist or if the ambguity is
        // over 20% and therefore the data is unreliable.
        if (result.isPresent() && lastResult.hasTargets() && lastResult.getBestTarget().getPoseAmbiguity() < 0.2) {

            SmartDashboard.putNumber("Best target ambguitiy", limeLight.getLatestResult().getBestTarget().getPoseAmbiguity());
    
            // Returns the data with the Pose3d, and the timestamp relative to the robot's time of when the pose was estimated at. (EstimatedRobotPose)
            return result.get();
        } else {
            return null;
        }
    }

    public AprilTagFieldLayout getApriltagLayout() {
        return apriltagLayout;
    }

}
