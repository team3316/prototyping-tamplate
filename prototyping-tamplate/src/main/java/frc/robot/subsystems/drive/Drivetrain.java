package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Module.ModuleLocation;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraIOPhotonVision;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.util.OdometryThread;

import static frc.robot.constants.DriveConstants.*;

public class Drivetrain extends SubsystemBase {
  private final Module[] modules;
  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Lock odometryLock = new ReentrantLock();

  private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];
  

  private Rotation2d rawGyroRotation = new Rotation2d();
  private final PoseEstimator poseEstimator;

  public Drivetrain(GyroIO gyro, ModuleIO... modules) {
    this.modules = new Module[modules.length];
    for (ModuleLocation p : ModuleLocation.values()) {
      lastModulePositions[p.ordinal()] = new SwerveModulePosition();
      int i = p.ordinal();
      this.modules[i] = new Module(p, modules[i]);
    }

    poseEstimator = new PoseEstimator(rawGyroRotation, isRed());

    OdometryThread.getInstance().start();

    this.gyro = gyro;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("headingPoseEstimated", this.getPose());
    odometryLock.lock();
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module m : modules) {
      m.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (Module m : modules) {
        m.stop();
      }

      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;


    //updates the odometry several times very periodic run
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (ModuleLocation pos : ModuleLocation.values()) {
        modulePositions[pos.ordinal()] = modules[pos.ordinal()].getOdometryPositions()[modules[pos.ordinal()].getOdometryPositions().length-1];
        moduleDeltas[pos.ordinal()] = new SwerveModulePosition(
            modulePositions[pos.ordinal()].distanceMeters - lastModulePositions[pos.ordinal()].distanceMeters,
            modulePositions[pos.ordinal()].angle);
      }

      //updates the poseestimator by the jro only if it is conected
      if (gyroInputs.isConnected) {
        rawGyroRotation = Rotation2d.fromDegrees(gyroInputs.odometryYawPositions[gyroInputs.odometryYawPositions.length-1]);
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateVision();
            
      poseEstimator.updateFromDrivetrain(sampleTimestamps[i], rawGyroRotation, modulePositions);


      
    }
    Logger.recordOutput("PoseEstimater/RobotPosePoseEstimator", this.getPose());

  }
  
  // Sets what alliance the robot is on to know how to flip the chassis speeds on the field relative mode
  public boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

  //For charactarization
  public void voltageDrive(Voltage voltMeasure) {
        for (Module module : modules) {
            module.io.driveByVoltage(voltMeasure.magnitude());
        }
    }
  

  //Runs as default command
  public void runVelocity(double xSpeed, double ySpeed, double rot, boolean fieldRelative){

    fieldRelative = fieldRelative && gyroInputs.isConnected;
    Logger.recordOutput("field relative", fieldRelative);
    SmartDashboard.putBoolean("field relative", fieldRelative);


    //field relative flip
    ChassisSpeeds speeds;
    if (fieldRelative) {
        if (!isRed()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, poseEstimator.getPose().getRotation());
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, poseEstimator.getPose().getRotation());
        }
    } else {
        speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }


    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DISCRETIZE_TIMESTEMP_DURATION);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_SPEED_METERS_PER_SECOND);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    

    for (ModuleLocation pos : ModuleLocation.values()) {
      modules[pos.ordinal()].runSetpoint(setpointStates[pos.ordinal()]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void stop() {
    runVelocity(0,0,0,false);
  }

  @AutoLogOutput(key="SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (ModuleLocation pos : ModuleLocation.values()) {
      states[pos.ordinal()] = modules[pos.ordinal()].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getPose();
  }

  public Rotation2d getRotation() {
    return poseEstimator.getPose().getRotation();
  }

  public void resetHeading() {
    poseEstimator.resetYaw();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState());
}

public void autoDrive(ChassisSpeeds speeds){
  runVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
}



  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  
  public void resetPose(Pose2d newPose) {
    this.poseEstimator.resetPose(newPose);
  }
}
