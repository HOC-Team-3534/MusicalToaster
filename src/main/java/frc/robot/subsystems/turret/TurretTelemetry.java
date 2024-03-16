package frc.robot.subsystems.turret;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.turret.Turret.TurretState;

public class TurretTelemetry {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable table = inst.getTable("Turret");
    private final DoublePublisher azimuth = table.getDoubleTopic("Azimuth").publish();
    private final DoublePublisher elevation = table.getDoubleTopic("Elevation").publish();

    private final DoublePublisher azimuthError = table.getDoubleTopic("Azimuth Error").publish();
    private final DoublePublisher elevationError = table.getDoubleTopic("Elevation Error").publish();
    private final DoublePublisher shooterError = table.getDoubleTopic("Shooter Error").publish();

    private final DoublePublisher robotRotationSinceBoot = table.getDoubleTopic("Robot Rotations Since Boot Degrees")
            .publish();
    private final DoublePublisher turretRotationSinceBoot = table.getDoubleTopic("Turret Rotations Since Boot Degrees")
            .publish();

    private final DoublePublisher initialRobotRotation = table.getDoubleTopic("Robot Initial Rotation Degrees")
            .publish();
    private final DoublePublisher initialTurretRotation = table.getDoubleTopic("Turret Initial Rotation Degrees")
            .publish();

    private final BooleanPublisher noteLoaded = table.getBooleanTopic("Note Loaded").publish();

    public TurretTelemetry() {
    }

    public void telemetrize(TurretState state) {
        azimuth.set(state.azimuth.getDegrees());
        elevation.set(state.elevation.getDegrees());

        azimuthError.set(state.rotateClosedLoopError.getDegrees());
        elevationError.set(state.tiltClosedLoopError.getDegrees());
        shooterError.set(state.shooterMotorClosedLoopError);

        robotRotationSinceBoot.set(state.robotRotationSinceBoot.getDegrees());
        turretRotationSinceBoot.set(state.turretRotationSinceBoot.getDegrees());

        initialRobotRotation.set(state.initialRobotRotation.getDegrees());
        initialTurretRotation.set(state.initialTurretRotation.getDegrees());

        noteLoaded.set(state.isNoteLoaded());
    }

}
