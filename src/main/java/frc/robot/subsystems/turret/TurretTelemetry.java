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

    private final BooleanPublisher noteLoaded = table.getBooleanTopic("Note Loaded").publish();

    private final BooleanPublisher tiltStillMonitor = table.getBooleanTopic("Tilt Still").publish();
    private final BooleanPublisher rotateStillMonitor = table.getBooleanTopic("Rotate Still").publish();

    private final DoublePublisher tiltVelocityOut = table.getDoubleTopic("Tilt Velocity Out").publish();
    private final DoublePublisher rotateVelocityOut = table.getDoubleTopic("Rotate Velocity Out").publish();

    private final DoublePublisher rotateSetpointPosition = table.getDoubleTopic("Rotate Setpoint Position").publish();
    private final DoublePublisher tiltSetpointPosition = table.getDoubleTopic("Tilt Setpoint Position").publish();

    public TurretTelemetry() {
    }

    public void telemetrize(TurretState state) {
        azimuth.set(state.azimuth.getDegrees());
        elevation.set(state.elevation.getDegrees());

        azimuthError.set(state.rotateClosedLoopError.getDegrees());
        elevationError.set(state.tiltClosedLoopError.getDegrees());
        shooterError.set(state.shooterMotorClosedLoopError);

        tiltVelocityOut.set(state.tiltVelocityOut);
        rotateVelocityOut.set(state.rotateVelocityOut);

        rotateSetpointPosition.set(state.rotatePositionSetpoint.getDegrees());
        tiltSetpointPosition.set(state.tiltPositionSetpoint.getDegrees());

        noteLoaded.set(state.isNoteLoaded());
    }

}
