package frc.robot.subsystems.turret;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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

    private final DoublePublisher tiltVelocityOut = table.getDoubleTopic("Tilt Velocity Out").publish();
    private final DoublePublisher rotateVelocityOut = table.getDoubleTopic("Rotate Velocity Out").publish();

    private final DoublePublisher rotateVelocityError = table.getDoubleTopic("Rotate Velocity Error").publish();
    private final DoublePublisher tiltVelocityError = table.getDoubleTopic("Tilt Velocity Error").publish();

    private final StringPublisher rotateState = table.getStringTopic("Rotate State").publish();
    private final StringPublisher tiltState = table.getStringTopic("Tilt State").publish();

    private final IntegerPublisher rotateStateOrdinal = table.getIntegerTopic("Rotate State Ordinal").publish();
    private final IntegerPublisher tiltStateOrdinal = table.getIntegerTopic("Tilt State Ordinal").publish();

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

        rotateVelocityError.set(state.rotateVelocityError.getDegrees());
        tiltVelocityError.set(state.tiltVelocityError.getDegrees());

        rotateState.set(state.rotateState);
        tiltState.set(state.tiltState);

        rotateStateOrdinal.set(state.rotateStateOrdinal);
        tiltStateOrdinal.set(state.tiltStateOrdinal);

        noteLoaded.set(state.isNoteLoaded());
    }

}
