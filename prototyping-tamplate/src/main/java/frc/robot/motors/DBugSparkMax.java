package frc.robot.motors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.PIDFGains;
import frc.robot.util.Within;

public class DBugSparkMax extends SparkMax {

    private SparkClosedLoopController _pidController;
    private RelativeEncoder _encoder;

    public DBugSparkMax(int deviceNumber) {
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

    private static SparkMaxConfig getConfigurator(
            PIDFGains gains, double positionFactor, double velocityFactor, int currentLimit) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(velocityFactor);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(gains.kP, gains.kI, gains.kD, gains.kF, ClosedLoopSlot.kSlot0)
                .iZone(gains.iZone)
                .outputRange(-gains.outputRange, gains.outputRange);
        config = configureLimitSwitches(config);
        config.smartCurrentLimit(currentLimit);
        config.voltageCompensation(12);
        config.idleMode(IdleMode.kBrake);
        config.openLoopRampRate(0.01);
        config.closedLoopRampRate(0.01);
        return config;
    }

    public static SparkMaxConfig configureLimitSwitches(SparkMaxConfig config){
        config.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        config.limitSwitch.forwardLimitSwitchEnabled(false);
        config.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
        config.limitSwitch.reverseLimitSwitchEnabled(false);
        return config;
    }

    public static DBugSparkMax create(
            int id, PIDFGains gains, double positionFactor, double velocityFactor, double position, int currentLimit) {
        DBugSparkMax sparkMax = new DBugSparkMax(id);
        SparkMaxConfig config = getConfigurator(gains, positionFactor, velocityFactor, currentLimit);
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sparkMax.setCANTimeout(50);
        sparkMax.setPosition(position);

        if (!sparkMax.verify(gains, positionFactor, velocityFactor)) {
            throw new IllegalStateException("SparkMax Failed to Initialize");
        }
        return sparkMax;
    }

    public static DBugSparkMax create(int id, PIDFGains gains, double positionFactor, double velocityFactor, double position) {
            return create(id, gains, positionFactor, velocityFactor, position, 40);
}

    public static DBugSparkMax create(int id, int currentLimit) {
        return create(id, new PIDFGains(0), 1, 1, 0, currentLimit);
    }

    public static DBugSparkMax create(int id) {
        return create(id, new PIDFGains(0), 1, 1, 0);
    }

    private SparkMaxConfig getCurrConfig() {
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
        int currentLimit = this.configAccessor.getSmartCurrentLimit();

        return getConfigurator(gains, positionFactor, velocityFactor, currentLimit);
    }

    public void updatPID(PIDFGains gains) {
        SparkMaxConfig config = getCurrConfig();

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(gains.kP, gains.kI, gains.kD, gains.kF)
                .iZone(gains.iZone)
                .outputRange(-gains.outputRange, gains.outputRange);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMeasurementPeriod(int period_ms) {
        SparkMaxConfig config = getCurrConfig();
        config.encoder.uvwMeasurementPeriod(period_ms);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAverageDepth(int depth) {
        SparkMaxConfig config = getCurrConfig();
        config.encoder.uvwAverageDepth(depth);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setInverted(boolean shouldInvert) {
        SparkMaxConfig config = getCurrConfig();
        config.inverted(shouldInvert);
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
