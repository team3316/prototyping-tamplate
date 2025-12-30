package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean isConnected = false;
        public AngularVelocity velocity = RadiansPerSecond.of(0);
        public Rotation2d heading = new Rotation2d();

        public double[] odometryYawTimestamps = new double[] {};
        public double[] odometryYawPositions = new double[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {};
}
