package frc.robot.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PIDFGains;

public class DBugTalon extends TalonFX {

    private TalonFXConfiguration _config;

    private DBugTalon(int id) {
        super(id);
        _config = new TalonFXConfiguration();
    }

    private static TalonFXConfiguration getTalonConfig(PIDFGains gains,
            double gearRatio,double currntLimit, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.withSensorToMechanismRatio(1 / gearRatio);

        config.Slot0.withKP(gains.kP)
                .withKI(gains.kI)
                .withKD(gains.kD)
                .withKV(gains.kF); // feedforward

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits
            .withSupplyCurrentLimit(currntLimit)
            .withSupplyCurrentLimitEnable(true);

        config.MotorOutput.withInverted(inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
            
        return config;
    }

    public void setForwardSoftLimit(double limit) {
        _config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(limit);
        _config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        this.getConfigurator().apply(_config);
    }

    public void setReverseSoftLimit(double limit) {
        _config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(limit);
        _config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
        this.getConfigurator().apply(_config);
    }

    public void updateByRemoteSensor(int sensorID) {
        _config.Feedback.withFeedbackRemoteSensorID(sensorID);
        _config.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
        this.getConfigurator().apply(_config);
    }

    public static DBugTalon create(int id, PIDFGains gains, double conversionFactor, double currentLimit, boolean inverted) {
        DBugTalon talon = new DBugTalon(id);
        TalonFXConfiguration configuration = getTalonConfig(gains, conversionFactor, currentLimit, inverted);
        talon.getConfigurator().apply(configuration);
        talon._config = configuration;
        return talon;
    }

    public static DBugTalon create(int id, PIDFGains gains, double conversionFactor, double currentLimit) {
        return create(id, gains, conversionFactor, currentLimit, false);
    }

    public static DBugTalon create(int id, PIDFGains gains, double conversionFactor) {
        return create(id, gains, conversionFactor, 40);
    }

    public static DBugTalon create(int id, PIDFGains gains) {
        return create(id, gains, 1);
    }

    public static DBugTalon create(int id) {
        return create(id, new PIDFGains(0));
    }
}
