package frc.robot.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class PhotonCam  extends SubsystemBase{
    private PhotonCamera cam;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator photonEstimator;
    private Swerve swerve;
    private PhotonPipelineResult result;
    private double lastEstTimestamp = 0;
    private Transform3d robotToCam;

    /**
     * Represents a Photon Camera used for vision processing
     * @param camName The name of the camera as defined from the photon vision website
     * @param robotToCam The 3d transformation from the center of the robot to the camera
     * @param swerve The swerve drive train
     * @param fieldLayout The AprilTagFieldLayout for the current game
     */
    public PhotonCam(
            String camName, 
            Transform3d robotToCam,
            Swerve swerve, 
            AprilTagFieldLayout fieldLayout){
        this.fieldLayout = fieldLayout;
        cam = new PhotonCamera(camName);
        this.swerve = swerve;
        this.robotToCam = robotToCam;

        photonEstimator = 
                new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Updates the PhotonCam sensor data and performs vision processing.
     * This method retrieves the latest result from the camera, estimates the robot's field relative pose
     * and then filters and adds the vision pose to the swerve drive pose estimator.
     * 
     * The robot pose is calculated, filtered, and added only if the vision result is present. 
     */
    public void update(){
        result = cam.getLatestResult();
        var visionEst = getEstimatedGlobalPose();
        visionEst.ifPresent(
            est -> {
                var target = result.getBestTarget();
                // robotPose is the estimated calculated pose using the field, apriltag pose, and robot pose
                Pose3d robotPose = 
                    PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(), 
                        fieldLayout.getTagPose(target.getFiducialId()).get(), 
                        robotToCam);

                filterAndAddVisionPose(est);
            });
    }

    /**
     * Returns an optional estimated global pose of the robot based on the vision estimator
     * Checks to see if there is a new result based off of the timestamp
     * 
     * @return An optional EstimatedRobotPose object representing the estimated global pose of the robot
     * or an empty optional if no estimated is available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
        var visionEstimation = photonEstimator.update(cam.getLatestResult());
        double latestTimestamp = cam.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult){
            lastEstTimestamp = latestTimestamp;
        }
        return visionEstimation;
    }

    /**
     * Filters and adds a vision pose to the robot's swerve drive system
     * The covariance matrix is the standard deviations of how much to trust the 
     * vision measurements. The higher the number, the more the measurements are trusted.
     * 
     * This method gets the average ambiguity and distance from all the targets seen
     * If the ambiguity or distance are above set thresholds, the pose estimation is not added
     * to the swerve drive pose estimator. 
     * 
     * Generally a ambiguity of .25 or lower is needed
     * 
     * @param pose The estimated robot pose
     */
    public void filterAndAddVisionPose(EstimatedRobotPose pose){
        Matrix<N3, N1> cov = VecBuilder.fill(0.4, 0.4, 0.4);

        double ambiguity = 0;
        double distance = 0;
        for (var tar : pose.targetsUsed){
            ambiguity += tar.getPoseAmbiguity() / pose.targetsUsed.size();
            distance += tar.getBestCameraToTarget().getTranslation().getNorm() / pose.targetsUsed.size();
        }
        if (ambiguity > 0.25 || distance > 10){
            return;
        }

        swerve.addVisionMeasurement(pose.estimatedPose, pose.timestampSeconds, cov);
    }

    
}
