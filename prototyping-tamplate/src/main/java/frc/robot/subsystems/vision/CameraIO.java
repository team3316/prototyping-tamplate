// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

/** Add your docs here. */
public interface CameraIO {

  @AutoLog
  public static class CameraIOInputs {
    public boolean hasResult = false;
    public double lastResultTimestamp = 0;
    public Pose3d cameraPose = new Pose3d();
    public double averageDistanceFromTags = 0;
    public int visibleTags = 0;
    public double poseAmbguity = 0;
    
  }
  public String getName();

  public void updateInputs(CameraIOInputs inputs);

  public boolean hasResult(Optional<EstimatedRobotPose> optionalEstimatedRobotPose);
}
