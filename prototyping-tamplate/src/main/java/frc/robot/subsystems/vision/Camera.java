// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Camera {
  private final CameraIO _io;

  private CameraIOInputsAutoLogged _inputs = new CameraIOInputsAutoLogged();

  public Camera(CameraIO io) {
    this._io = io;
  }


  
  public void update() {
    _io.updateInputs(_inputs);

    Logger.processInputs("Vision/"+ this._io.getName(), _inputs);
  }

  public Pose3d getCameraPose() {
    if (!hasValidresults()) {
      return null;
    }
    return _inputs.cameraPose;
  }

  public double getAverageDistanceFromTags() {
    return _inputs.averageDistanceFromTags;
  }

  public int getVisibleTags() {
    return _inputs.visibleTags;
  }

  public double getTimestemp() {
    return _inputs.lastResultTimestamp;
  }

  public boolean hasValidresults() {
    return (_inputs.hasResult && _inputs.poseAmbguity < VisionConstants.MAXIMUM_AMBIGUITY);
  }

  public String getName(){
    return _io.getName();
  }
}
