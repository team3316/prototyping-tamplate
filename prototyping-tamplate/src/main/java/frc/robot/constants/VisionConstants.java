package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */
public class VisionConstants {

    public static final double maxYDistanceFromReef = 1.0; // meters, max y distance to reef for which we allow the arm to adjust according to x distance

    public static final double distanceToReefThreshhold = 0; // meters


  public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

  static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  static {
    for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
      TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
  }

  public static final double MAXIMUM_AMBIGUITY = 0.1;

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final PoseStrategy MAIN_STRATEGY =
      PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

      public static final Translation3d CAM1_TRANSLATION3D = new Translation3d(-0.25131945, -0.22117854, 0.36375851);
      public static final Rotation3d CAM1_ROTATION3D =  new Rotation3d(Math.toRadians(0), Math.toRadians(15.799217), Math.toRadians(180 - 31));
      public static final String CAM1_NAME = "left-reef-camera";

      public static final Translation3d CAM2_TRANSLATION3D = new Translation3d(-0.25131945, 0.22117854, 0.36375851);
      public static final Rotation3d CAM2_ROTATION3D =  new Rotation3d(Math.toRadians(0),Math.toRadians(15.799217),Math.toRadians(180 + 31));
      public static final String CAM2_NAME = "right-reef-camera";

      public static final double TRANSLATIONS_STD_EXPONENT = 0.5;
      public static final double THETA_STD_EXPONENT = 0.05;

    public static final double STATE_XY_STD = 0.05;
    public static final double STATE_THETA_STD = Math.toRadians(5);

    public static final String intakeCameraName = "intake";
    public static final Transform3d robotToIntakeCamera = new Transform3d(
        -0.153,
        0,
        0.925,
        new Rotation3d(0, Units.degreesToRadians(37), 0)
    );

    public static final int coralId = 1;
    public static final int algaeId = 0;
}
