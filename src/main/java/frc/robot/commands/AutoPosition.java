package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPosition {
    final Translation2d position;
    final AutoPositionType type;

    public AutoPosition(Translation2d position, AutoPositionType type) {
        this.position = position;
        this.type = type;
    }

    public Translation2d getPosition() {
        return position;
    }

    public AutoPositionType getType() {
        return type;
    }

    public Translation2d minus(AutoPosition otherPosition) {
        return position.minus(otherPosition.getPosition());
    }

    public enum AutoPositionType {
        Note,
        Shoot
    }
}
