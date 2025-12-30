package frc.robot.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.util.PIDFGains;
import frc.robot.util.Within;

public class DBugSparkFlex extends SparkFlex {

    private SparkClosedLoopController _pidController;
    private RelativeEncoder _encoder;

    public DBugSparkFlex(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);

        this._encoder = this.getEncoder();

        this._pidController = this.getClosedLoopController();
    }

    public void setReference(
            double value,
            ControlType ctrl,
            ClosedLoopSlot pidSlot,
            double arbFeedforward,
            SparkClosedLoopController.ArbFFUnits arbFFUnits) {
        this._pidController.setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    }

    public void setReference(double value, ControlType ctrl) {
        this._pidController.setReference(value, ctrl);
    }

    public void setPosition(double value) {
        this._encoder.setPosition(value);
    }

    public double getVelocity() {
        return this._encoder.getVelocity();
    }

    public double getPosition() {
        return this._encoder.getPosition();
    }

    private static SparkFlexConfig getConfigurator(
            PIDFGains gains, double positionFactor, double velocityFactor) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(gains.kP, gains.kI, gains.kD, gains.kF)
                .iZone(gains.iZone)
                .outputRange(-gains.outputRange, gains.outputRange);
        config.smartCurrentLimit(40);
        config.voltageCompensation(12);
        config.idleMode(IdleMode.kBrake);
        config.openLoopRampRate(0.01);
        config.closedLoopRampRate(0.01);
        return config;
    }

    public static DBugSparkFlex create(
            int id, PIDFGains gains, double positionFactor, double velocityFactor, double position) {
        DBugSparkFlex sparkFlex = new DBugSparkFlex(id);
        SparkFlexConfig config = getConfigurator(gains, positionFactor, velocityFactor);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sparkFlex.setCANTimeout(50);
        sparkFlex.setPosition(position);

        if (!sparkFlex.verify(gains, positionFactor, velocityFactor)) {
            throw new IllegalStateException("SparkFlex Failed to Initialize");
        }
        return sparkFlex;
    }

    public static DBugSparkFlex create(int id) {
        return create(id, new PIDFGains(0), 1, 1, 0);
    }

    public static DBugSparkFlex create(int id, double openLoopRampRate) {
        DBugSparkFlex sparkFlex = create(id);
        SparkFlexConfig config = getConfigurator(new PIDFGains(0), 1, 1);
        config.openLoopRampRate(openLoopRampRate);
        sparkFlex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return sparkFlex;
    }

    private SparkFlexConfig getCurrConfig() {
        ClosedLoopConfigAccessor cLoopConfigAccessor = this.configAccessor.closedLoop;
        PIDFGains gains = new PIDFGains(
                cLoopConfigAccessor.getP(),
                cLoopConfigAccessor.getI(),
                cLoopConfigAccessor.getD(),
                cLoopConfigAccessor.getFF(),
                cLoopConfigAccessor.getMaxOutput(),
                cLoopConfigAccessor.getIZone());
        double positionFactor = this.configAccessor.encoder.getPositionConversionFactor();
        double velocityFactor = this.configAccessor.encoder.getVelocityConversionFactor();
        return getConfigurator(gains, positionFactor, velocityFactor);
    }

    public void updatPID(PIDFGains gains) {
        SparkFlexConfig config = getCurrConfig();

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(gains.kP, gains.kI, gains.kD, gains.kF)
                .iZone(gains.iZone)
                .outputRange(-gains.outputRange, gains.outputRange);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMeasurementPeriod(int period_ms) {
        SparkFlexConfig config = getCurrConfig();
        config.encoder.uvwMeasurementPeriod(period_ms);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAverageDepth(int depth) {
        SparkFlexConfig config = getCurrConfig();
        config.encoder.uvwAverageDepth(depth);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public int getMeasurementPeriod() {
        return this.configAccessor.encoder.getUvwMeasurementPeriod();
    }

    public boolean verify(PIDFGains gains, double positionFactor, double velocityFactor) {
        if (!Within.range(gains.kP, this.configAccessor.closedLoop.getP(), 0.001))
            return false;
        if (!Within.range(gains.kI, this.configAccessor.closedLoop.getI(), 0.001))
            return false;
        if (!Within.range(gains.kD, this.configAccessor.closedLoop.getD(), 0.001))
            return false;
        if (!Within.range(gains.kF, this.configAccessor.closedLoop.getFF(), 0.001))
            return false;
        if (!Within.range(
                positionFactor, this.configAccessor.encoder.getPositionConversionFactor(), 0.001))
            return false;
        if (!Within.range(
                velocityFactor, this.configAccessor.encoder.getVelocityConversionFactor(), 0.001))
            return false;
        return true;
    }
}
