package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DriveConstants;

public class OdometryThread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static OdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    private Lock lock = new ReentrantLock();

    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread() {
        notifier.setName("OdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQ_HZ);
        }
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        lock.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            lock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        lock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            lock.unlock();
        }
        return queue;
    }

    private void run() {
        lock.lock();
        try {
            double timestamp = RobotController.getFPGATime() / 1e6; // Seconds

            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
            for (Queue<Double> q : timestampQueues) {
                q.offer(timestamp);
            }
        } finally {
            lock.unlock();
        }
    }
}
