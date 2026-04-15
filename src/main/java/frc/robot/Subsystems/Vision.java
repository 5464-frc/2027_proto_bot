package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public static final AprilTagFieldLayout kTagField = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
    public class collectiveCamera {
        PhotonCamera cam;
        PhotonPoseEstimator estimator;
        Double oldestPoseSeconds;
        List<PhotonPipelineResult> resultCache = new ArrayList<>();
        Optional<EstimatedRobotPose> pose;
        List<EstimatedRobotPose> poseCache = new ArrayList<>();

        collectiveCamera(String name, PhotonPoseEstimator estimator, Double oldestPoseSeconds) {
            this.cam = new PhotonCamera(name);
            this.cam.setFPSLimit(50); // prevent buildup
            this.estimator = estimator;
            this.oldestPoseSeconds = oldestPoseSeconds;
        }

        public void update() {
            resultCache = this.cam.getAllUnreadResults();

            if (resultCache.size() > 1) {
                System.err.println("cache buildup (check 20ms threshhold)");
            } else if (resultCache.isEmpty()) {
                return;
            }

            if (!resultCache.get(0).hasTargets()) {
                return;
            }

            this.pose = this.estimator.estimateCoprocMultiTagPose(resultCache.get(0));

            if (this.pose.isEmpty()) {
                // backup method
                this.pose = this.estimator.estimateAverageBestTargetsPose(resultCache.get(0));
                if (this.pose.isEmpty()) {
                    return;
                }
            }

            this.poseCache.add(this.pose.get());

            for (int i = 0; i < this.poseCache.size(); i++) {
                if (this.poseCache.get(i).timestampSeconds > this.oldestPoseSeconds) {
                    this.poseCache.remove(i);
                }
            }
        }
    }
    @Override
    public void periodic() {
    }
}
