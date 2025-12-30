package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.OdometryThread;

import static edu.wpi.first.units.Units.*;

import java.util.Queue;

public class GyroIOPigeon implements GyroIO {
  private final PigeonIMU imu;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> yawQueue;

  double rates[] = new double[3];

  public GyroIOPigeon(int port) {
    imu = new PigeonIMU(port);

    timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    yawQueue = OdometryThread.getInstance().registerSignal(imu::getFusedHeading);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = imu.getState() == PigeonState.Ready;
    inputs.heading = getRotation2d();

    // Angular velocity on the Pigeon 1 is funky
    imu.getRawGyro(rates);
    inputs.velocity = DegreesPerSecond.of(rates[2]);

    // Cursed java streams
    inputs.odometryYawTimestamps = timestampQueue.stream().mapToDouble((x) -> x).toArray();
    inputs.odometryYawPositions = yawQueue.stream().mapToDouble((x) -> x).toArray();

    timestampQueue.clear();
    yawQueue.clear();
  }

  private Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(imu.getFusedHeading());
  }
}
