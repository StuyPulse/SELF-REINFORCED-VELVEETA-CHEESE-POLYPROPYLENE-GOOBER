package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends AprilTagVision {

    private final PhotonCamera[] cameras;
    private final boolean[] enabled;
    private final PhotonPoseEstimator[] poseEstimators;
    private final ArrayList<VisionData> outputs;

    private final FieldObject2d robot;

    protected PhotonVision() {
        this.cameras = new PhotonCamera[Cameras.APRILTAG_CAMERAS.length];
        for (int i = 0; i < Cameras.APRILTAG_CAMERAS.length; i++) {
            cameras[i] = new PhotonCamera(Cameras.APRILTAG_CAMERAS[i].getName());
        }

        enabled = new boolean[Cameras.APRILTAG_CAMERAS.length];

        for (int i = 0; i < enabled.length; i++) {
            enabled[i] = true;
        }

        poseEstimators = new PhotonPoseEstimator[Cameras.APRILTAG_CAMERAS.length];
        for (int i = 0; i < Cameras.APRILTAG_CAMERAS.length; i++) {
            poseEstimators[i] = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), 
                PoseStrategy.AVERAGE_BEST_TARGETS, 
                Cameras.APRILTAG_CAMERAS[i].getLocation().minus(new Pose3d())
                );
        }

        outputs = new ArrayList<VisionData>();

        robot = SwerveDrive.getInstance().getField().getObject("Vision Pose");
    }

    /**
     * Returns the outputs of the vision system.
     *
     * @return the outputs of the vision system
     */
    @Override
    public ArrayList<VisionData> getOutputs() {
        return outputs;
    }

    /**
     * Sets the tag layout of the vision system.
     *
     * @param ids the tag IDs
     */
    @Override
    public void setTagWhitelist(int... ids) {
        
    }

    @Override
    public void setCameraEnabled(String name, boolean enabled) {
        for (int i = 0; i < Cameras.APRILTAG_CAMERAS.length; i++) {
            if (cameras[i].getName().equals(name)) {
                this.enabled[i] = enabled;
            }
        }
    }

    private int[] getIDs(PhotonPipelineResult pipelineResult) {
        ArrayList<Integer> ids = new ArrayList<Integer>();
        for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
            ids.add(target.getFiducialId());
        }
        return ids.stream().mapToInt(i -> i).toArray();
    }

    @Override
    public void periodic() {
        super.periodic();

        outputs.clear();

        for (int i = 0; i < cameras.length; i++) {
            final int index = i;
            if (enabled[index]) {
                PhotonPipelineResult latestResult = cameras[index].getLatestResult();
                Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimators[index].update(latestResult);
                if (latestResult.hasTargets()) {
                    if (latestResult.getBestTarget().getPoseAmbiguity() < Settings.Vision.POSE_AMBIGUITY_RATIO_THRESHOLD) {
                        estimatedRobotPose.ifPresent(
                            (EstimatedRobotPose robotPose) -> {
                                VisionData data = new VisionData(robotPose.estimatedPose, getIDs(latestResult), robotPose.timestampSeconds, latestResult.getBestTarget().getArea());
                                outputs.add(data);
                                updateTelemetry("Vision/" + cameras[index].getName(), data);
                            }
                        );
                    }
                }
            }
        }

        SmartDashboard.putBoolean("Vision/Has Any Data", outputs.size() > 0);
    }

    private void updateTelemetry(String prefix, VisionData data) {
        SmartDashboard.putNumber(prefix + "/Pose X", data.getPose().getX());
        SmartDashboard.putNumber(prefix + "/Pose Y", data.getPose().getY());
        SmartDashboard.putNumber(prefix + "/Pose Z", data.getPose().getZ());
        SmartDashboard.putNumber(prefix + "/Pose Rotation", Units.radiansToDegrees(data.getPose().getRotation().getAngle()));

        SmartDashboard.putNumber(prefix + "/Distance to Tag", data.getDistanceToPrimaryTag());

        SmartDashboard.putNumber(prefix + "/Timestamp", data.getTimestamp());

        robot.setPose(data.getPose().toPose2d());
    }
}
