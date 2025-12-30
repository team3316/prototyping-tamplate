package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public Distance drivePosition = Meters.of(0);
        public LinearVelocity driveVelocityMetersPerSec = MetersPerSecond.of(0);
        public double driveVelocityRPM = 0;
        public Voltage driveAppliedVoltage = Volts.of(0);
        public Current driveCurrent = Amps.of(0);

        public boolean steerConnected = false;
        public Rotation2d steerPosition = new Rotation2d();
        public AngularVelocity steerVelocity = RadiansPerSecond.of(0);
        public Voltage steerAppliedVoltage = Volts.of(0);
        public Current steerCurrent = Amps.of(0);

        public Rotation2d absSteerRotation = new Rotation2d();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRot = new double[] {};
        public double[] odometrySteerPositionsDeg = new double[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {};

    public default void setDriveVelocity(double vel) {};

    public default void setSteerPosition(Rotation2d rotation) {};

    public default void driveByVoltage(double Voltage) {};
    public default void dontMove(){};

    public default void stop() {};
}
