package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer.ShooterType;

public class AutoPosition {
    final Translation2d position;
    final AutoPositionType type;
    final ShooterType shootOrStealNote;
    boolean skippable = true;

    public AutoPosition(Translation2d position, AutoPositionType type, ShooterType shootOrStealNote) {
        this.position = position;
        this.type = type;
        this.shootOrStealNote = shootOrStealNote;
    }

    public Translation2d getPosition() {
        return position;
    }

    public AutoPositionType getType() {
        return type;
    }

    public ShooterType getShootOrStealNote() {
        return shootOrStealNote;
    }

    public AutoPosition withNotSkippable() {
        this.skippable = false;
        return this;
    }

    public boolean isSkippable() {
        return this.skippable;
    }

    public Translation2d minus(AutoPosition otherPosition) {
        return position.minus(otherPosition.getPosition());
    }

    public enum AutoPositionType {
        Note,
        Shoot
    }
}
