package frc.robot.subsystems.drive;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants.SwerveModuleConfig;
import frc.robot.motors.DBugCANcoder;
import frc.robot.motors.DBugSparkMax;
import frc.robot.subsystems.drive.Module.ModuleLocation;
import frc.robot.util.OdometryThread;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.DriveConstants.*;

public class ModuleIOSpark implements ModuleIO {

    
    private final DBugSparkMax drive;
    private final DBugSparkMax steer;

    private final DBugCANcoder absEncoder;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steerPositionQueue;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer steerConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSpark(ModuleLocation module) {
        SwerveModuleConfig cfg = MODULES[module.ordinal()];

        absEncoder = DBugCANcoder.create(cfg.CANCoder_ID, cfg.ZERO_ROTATION);

        drive = DBugSparkMax.create(cfg.DRIVE_ID, DRIVE_GAINS, DRIVE_POSITION_FACTOR, DRIVE_VELOCITY_FACTOR, 0, (int)DRIVE_CURRENT_LIMIT);
        steer = DBugSparkMax.create(cfg.STEER_ID, STEER_GAINS, STEER_POSITION_FACTOR, STEER_VELOCITY_FACTOR, absEncoder.getAbsolutePosition().getValueAsDouble() * 360,STEER_CURRENT_LIMIT);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(drive::getPosition);
        steerPositionQueue = OdometryThread.getInstance().registerSignal(absEncoder::getAbsolutePositionAsDouble);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveConnected = driveConnectedDebounce.calculate(drive.hasStickyFault());
        inputs.driveAppliedVoltage = Volts.of(drive.getAppliedOutput() * drive.getBusVoltage());
        inputs.driveCurrent = Amps.of(drive.getOutputCurrent());
        inputs.drivePosition = Meters.of(drive.getPosition());

        Logger.recordOutput("driveVel", drive.getVelocity()*DRIVE_VELOCITY_FACTOR);
        inputs.driveVelocityMetersPerSec = MetersPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(drive.getVelocity()) * (WHEEL_DIAMETER_M/2));
        inputs.driveVelocityRPM = drive.getVelocity();
        
        inputs.steerConnected = steerConnectedDebounce.calculate(steer.hasStickyFault());
        inputs.steerAppliedVoltage = Volts.of(steer.getAppliedOutput() * steer.getBusVoltage());
        inputs.steerCurrent = Amps.of(steer.getOutputCurrent());
        inputs.steerPosition = Rotation2d.fromDegrees(steer.getPosition());
        inputs.steerVelocity = DegreesPerSecond.of(steer.getVelocity());

        inputs.absSteerRotation = Rotation2d.fromDegrees(absEncoder.getAbsolutePositionAsDouble());

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double x) -> x).toArray();
        inputs.odometryDrivePositionsRot = drivePositionQueue.stream().mapToDouble((Double x) -> x).toArray();
        inputs.odometrySteerPositionsDeg = steerPositionQueue.stream().mapToDouble((Double x) -> x).toArray();

        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveVelocity(double velocity) {
        drive.setReference(velocity, ControlType.kVelocity);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        Logger.recordOutput("rotation.getDegrees", rotation.getDegrees());
        steer.setReference(MathUtil.inputModulus(rotation.getDegrees(), 0.0, 360), ControlType.kPosition);
    }

    @Override
    public void driveByVoltage(double Voltage){
        drive.setVoltage(Voltage);
    }

    @Override
    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }
}
