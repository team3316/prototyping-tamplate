package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.Camera;

public class PoseEstimator extends SubsystemBase{
    private ArrayList<Camera> _cameras;
    // private SwerveDriveOdometry _odometry;
    // private Supplier<Integer> _tagIDSupplier;
    private boolean isRed;
    

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    private static final Vector<N3> visionStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private final SwerveDrivePoseEstimator poseEstimator;

    public PoseEstimator(Supplier<Integer> tagIDSupplier, Rotation2d gyroAngle, boolean isRed) {
        var modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = new SwerveModulePosition();
        }
        // this._tagIDSupplier = tagIDSupplier;
        this.poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            gyroAngle,
            modulePositions,
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);
        this._cameras = new ArrayList<>();
        this.isRed = isRed;
    }

    public PoseEstimator(Rotation2d gyroAngle, boolean isRed){
        var modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = new SwerveModulePosition();
        }

        this.poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            gyroAngle,
            modulePositions,
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);
        this._cameras = new ArrayList<>();
        this.isRed = isRed;
    }

    public void addValidVisionMeasurement(Pose3d estimatedPose, double timestampSeconds) {
        if (estimatedPose == null) {
            return;
        }
        Logger.recordOutput("camera pose", estimatedPose);
        poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), timestampSeconds);
    }

    public void addCamera(Camera newCam) {
        this._cameras.add(newCam);
    }

    public void updateVision() {
        for (Camera cam : _cameras) {
            cam.update();
            updateStdDevs(cam.getAverageDistanceFromTags(), cam.getVisibleTags());
            Logger.recordOutput("Position:" + cam.getName(), cam.getCameraPose());


            addValidVisionMeasurement(cam.getCameraPose(), cam.getTimestemp());
        }
    }

    private void updateStdDevs(double averageDistance, int visibleTags) {
        final double translationStd = VisionConstants.TRANSLATIONS_STD_EXPONENT
                * Math.pow(averageDistance, 2)
                / (visibleTags * visibleTags);
        final double thetaStd = VisionConstants.THETA_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;

        Matrix<N3, N1> newVals = VecBuilder.fill(translationStd, translationStd, thetaStd);
        Logger.recordOutput("vision std val", newVals.getData());
        

        poseEstimator.setVisionMeasurementStdDevs(newVals);
    }

    public void updateFromDrivetrain(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions){
        this.poseEstimator.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
        // this._odometry.update(gyroAngle, wheelPositions);
    }
    

    public void resetPose(Pose2d newPose) {
        this.poseEstimator.resetPose(newPose);
    }

    // public void poseEstimatorToOdometry(){
    //     this._odometry.resetPose(getPose());
    // }


    @AutoLogOutput
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }


    public void resetYaw() {
        this.poseEstimator.resetRotation(new Rotation2d(Math.toRadians(isRed ? 180 : 0)));
    }

}

