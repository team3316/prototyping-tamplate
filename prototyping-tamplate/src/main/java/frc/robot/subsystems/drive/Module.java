package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static frc.robot.constants.DriveConstants.WHEEL_DIAMETER_M;

public class Module {

  // The modoule IDs represented by this enum
  public enum ModuleLocation {
    FL, FR, BL, BR;
  }

  public final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleLocation pos;

  private SwerveModulePosition[] odometryPositions;

  public Module(ModuleLocation pos, ModuleIO io) {
    this.pos = pos;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + pos.ordinal(), inputs);

    // Runs several times in each periodic
    int sampleCount = inputs.odometryTimestamps.length;

    if (inputs.odometryDrivePositionsRot.length == 0 || inputs.odometrySteerPositionsDeg.length == 0 || inputs.odometryTimestamps.length == 0) return;

    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < odometryPositions.length; i++) {
      odometryPositions[i] = new SwerveModulePosition(inputs.odometryDrivePositionsRot[inputs.odometryDrivePositionsRot.length - 1] * Math.PI * WHEEL_DIAMETER_M,
          Rotation2d.fromDegrees(inputs.odometrySteerPositionsDeg[inputs.odometrySteerPositionsDeg.length - 1]));
    }
  }

  public void stop() {
    io.stop();
  };

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public void runSetpoint(SwerveModuleState setpoint1) {
    
    SwerveModuleState setpoint = optimize(setpoint1, inputs.steerPosition.getDegrees());

    setpoint.cosineScale(inputs.steerPosition);
    if (setpoint.speedMetersPerSecond == 0)io.dontMove();
    else io.setDriveVelocity(setpoint.speedMetersPerSecond);
    io.setSteerPosition(setpoint.angle);
    
  }

  public Rotation2d getAngle() {
    return inputs.absSteerRotation;
  }

  public Distance getDistance() {
    return inputs.drivePosition;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }



  public LinearVelocity getVelocity() {
    return inputs.driveVelocityMetersPerSec;
  }

  //the DBug module angle optimization method
  private static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
    // desired angle diff in [-360, +360]
    double _angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;

    double targetAngle = currentAngle + _angleDiff;
    double targetSpeed = desiredState.speedMetersPerSecond;

    // Q1 undershot. We expect a CW turn.
    if (_angleDiff <= -270)
        targetAngle += 360;

    // Q2 undershot. We expect a CCW turn to Q4 & reverse direction.
    // Q3. We expect a CW turn to Q1 & reverse direction.
    else if (-90 > _angleDiff && _angleDiff > -270) {
        targetAngle += 180;
        targetSpeed = -targetSpeed;
    }

    // Q2. We expect a CCW turn to Q4 & reverse direction.
    // Q3 overshot. We expect a CW turn to Q1 & reverse direction.
    else if (90 < _angleDiff && _angleDiff < 270) {
        targetAngle -= 180;
        targetSpeed = -targetSpeed;
    }

    // Q4 overshot. We expect a CCW turn.
    else if (_angleDiff >= 270)
        targetAngle -= 360;

    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
}
