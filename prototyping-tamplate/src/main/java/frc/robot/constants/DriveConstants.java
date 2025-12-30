package frc.robot.constants;

import frc.robot.subsystems.drive.Module.ModuleLocation;
import frc.robot.util.PIDFGains;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

public class DriveConstants {
    public static class SwerveModuleConfig {
        public final double ZERO_ROTATION;
        public final Translation2d TRANSLATION;
        public final int DRIVE_ID, STEER_ID, CANCoder_ID;
        public final ModuleLocation MODULE;

        public SwerveModuleConfig(ModuleLocation position, double zeroRotationDeg, Translation2d translation, int driveId, int steerId, int CANCoderId) {
            this.MODULE = position;
            this.ZERO_ROTATION = zeroRotationDeg;
            this.TRANSLATION = translation;
            this.DRIVE_ID = driveId;
            this.STEER_ID = steerId;
            this.CANCoder_ID = CANCoderId;
        }
    }

    public static final double MAX_ROTATION_SPEED_RAD_PER_SEC = 11.5;
  
    public static final double CANCoderFL_OFFSET = Units.radiansToRotations(1.620); //rot
    public static final double CANCoderFR_OFFSET = Units.radiansToRotations(5.203); //rot
    public static final double CANCoderBL_OFFSET = Units.radiansToRotations(6.260); //rot
    public static final double CANCoderBR_OFFSET = Units.radiansToRotations(1.175); //rot

    // A rectangle describing the arrangement of modules
    public static final double MODULES_WIDTH_M = 0.572;
    public static final double MODULES_LENGTH_M = 0.531;


    //modules configs
    public static final SwerveModuleConfig[] MODULES = new SwerveModuleConfig[] {
        new SwerveModuleConfig(
            ModuleLocation.FL,
            CANCoderFL_OFFSET,
            new Translation2d(MODULES_WIDTH_M / 2.0, MODULES_LENGTH_M / 2.0),
            1, 2, 3
        ),
        new SwerveModuleConfig(
            ModuleLocation.FR,
            CANCoderFR_OFFSET,
            new Translation2d(MODULES_WIDTH_M / 2.0, -MODULES_LENGTH_M / 2.0),
            4, 5, 6 
        ),
        new SwerveModuleConfig(
            ModuleLocation.BL,
            CANCoderBL_OFFSET,
            new Translation2d(-MODULES_WIDTH_M / 2.0, MODULES_LENGTH_M / 2.0),
            7, 8, 9 
        ),
        new SwerveModuleConfig(
            ModuleLocation.BR,
            CANCoderBR_OFFSET,
            new Translation2d(-MODULES_WIDTH_M / 2.0, -MODULES_LENGTH_M / 2.0),
            10, 11, 12 
        )
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        MODULES[0].TRANSLATION,
        MODULES[1].TRANSLATION,
        MODULES[2].TRANSLATION,
        MODULES[3].TRANSLATION
        );

    public static final double ODOMETRY_FREQ_HZ = 100.0;

    public static final double DRIVE_GEARING = 1.0 / 8.14;
    public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4);

    public static final double DRIVE_CURRENT_LIMIT = 80.0;
    public static final double DRIVE_POSITION_FACTOR = DRIVE_GEARING * WHEEL_DIAMETER_M * Math.PI; // Rotations -> Meters
    public static final double DRIVE_VELOCITY_FACTOR = DRIVE_POSITION_FACTOR / 60.0; // RPM -> m/s

    //drive PIDF
    
    public static final double driveKp = 4.6535; // .612; // in seconds per meter
    public static final double driveKi = 0.0;
    public static final double driveKd = 0.0; // in seconds per meter
    public static final double driveKf = 3.0866; // m/s / voltage
    public static final PIDFGains DRIVE_GAINS = new PIDFGains(driveKp,driveKi,driveKd,driveKf);

    

    public static final double STEER_GEARING = 1.0 / 12.8;

    public static final int STEER_CURRENT_LIMIT = 60;
    public static final double STEER_POSITION_FACTOR = STEER_GEARING * 360; // Rotations -> Degrees
    public static final double STEER_VELOCITY_FACTOR = STEER_POSITION_FACTOR / 60.0; // RPM -> deg/s

    //steering PID
    public static final double steeringKp = 0.004; // in 1 / wheel degrees
    public static final PIDFGains STEER_GAINS = new PIDFGains(steeringKp);

    public static final double DISCRETIZE_TIMESTEMP_DURATION = 0.02;





    public static final int GYRO_PORT = 13;

    public static final double MAX_SPEED_METERS_PER_SECOND = 5; // TODO measure
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 10; // TODO measure
}