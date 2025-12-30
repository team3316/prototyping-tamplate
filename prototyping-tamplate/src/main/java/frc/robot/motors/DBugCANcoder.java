package frc.robot.motors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class DBugCANcoder extends CANcoder {

    public DBugCANcoder(int id) {
        super(id);
    }

    public static DBugCANcoder create(int id, double zeroAngle, boolean shouldInvert, double discontinuityPoint) {
        CANcoderConfiguration _absencoderConfig = new CANcoderConfiguration();
        DBugCANcoder CANcoder = new DBugCANcoder(id);

        // Always set CANcoder relative encoder to 0 on boot

        // Configure the offset angle of the magnet
        _absencoderConfig.MagnetSensor.withMagnetOffset((1 - zeroAngle));
        _absencoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(discontinuityPoint);

        if (shouldInvert) {
            _absencoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        }
        final double configTimeoutSeconds = 0.3;
        CANcoder.getConfigurator().apply(_absencoderConfig, configTimeoutSeconds);

        return CANcoder;
    }

    public static DBugCANcoder create(int id, double zeroAngle, boolean shouldInvert) {
        return create(id, zeroAngle, shouldInvert, 1);
    }

    public static DBugCANcoder create(int id, double zeroAngle) {
        return create(id, zeroAngle, false);
    }

    public double getAbsolutePositionAsDouble(){
        return getAbsolutePosition().getValueAsDouble()*360;
    }
}
