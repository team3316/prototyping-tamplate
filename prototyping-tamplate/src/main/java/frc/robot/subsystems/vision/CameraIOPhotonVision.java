// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class CameraIOPhotonVision implements CameraIO {
  PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Pose3d emptyPose3d = new Pose3d();

  public CameraIOPhotonVision(String camraName, Transform3d robotToCamra) {

    this.camera = new PhotonCamera(camraName);

    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.FIELD_LAYOUT, VisionConstants.MAIN_STRATEGY, robotToCamra);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    final List<PhotonPipelineResult> latestResults = camera.getAllUnreadResults();
    Logger.recordOutput("cam results list length", String.valueOf(latestResults.size()));
    for (PhotonPipelineResult latestResult : latestResults) {

      Optional<EstimatedRobotPose> optionalEstimatedRobotPose =
          photonPoseEstimator.update(latestResult);

      inputs.hasResult = hasResult(optionalEstimatedRobotPose);
      if (inputs.hasResult) {
        final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();
        inputs.cameraPose = estimatedRobotPose.estimatedPose;
        inputs.lastResultTimestamp = estimatedRobotPose.timestampSeconds;
        inputs.visibleTags = estimatedRobotPose.targetsUsed.size();
        inputs.averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
        inputs.poseAmbguity = latestResult.getBestTarget().poseAmbiguity;
      } else {
        inputs.visibleTags = 0;
        inputs.cameraPose = emptyPose3d;
      }

      logVisibleTags(inputs.hasResult, optionalEstimatedRobotPose);
    }
  }

  private double getAverageDistanceFromTags(PhotonPipelineResult result) {
    final List<PhotonTrackedTarget> targets = result.targets;
    double distanceSum = 0;

    for (PhotonTrackedTarget currentTarget : targets) {
      final Translation2d distanceTranslation =
          currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
      distanceSum += distanceTranslation.getNorm();
    }

    return distanceSum / targets.size();
  }


  private void logVisibleTags(
      boolean hasResult, Optional<EstimatedRobotPose> optionalEstimatedRobotPose) {
    if (!hasResult) {
      Logger.recordOutput("VisibleTags/" + camera.getName(), new Pose2d[0]);
      return;
    }

    final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();
    final Pose2d[] visibleTagPoses = new Pose2d[estimatedRobotPose.targetsUsed.size()];
    for (int i = 0; i < visibleTagPoses.length; i++) {
      final int currentId = estimatedRobotPose.targetsUsed.get(i).getFiducialId();
      final Pose2d currentPose = VisionConstants.TAG_ID_TO_POSE.get(currentId).toPose2d();
      visibleTagPoses[i] = currentPose;
    }
    Logger.recordOutput("VisibleTags/" + camera.getName(), visibleTagPoses);
  }

  @Override
  public boolean hasResult(Optional<EstimatedRobotPose> optionalEstimatedRobotPose) {
    final boolean isEmpty = optionalEstimatedRobotPose.isEmpty();
    return !isEmpty;
  }

  @Override
  public String getName(){
    return this.camera.getName();
  }
}
