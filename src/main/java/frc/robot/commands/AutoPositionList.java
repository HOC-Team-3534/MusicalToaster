package frc.robot.commands;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer.ShooterType;
import frc.robot.commands.AutoPosition.AutoPositionType;

public class AutoPositionList extends LinkedList<AutoPosition> {

    static final double SHIFT_FOR_3_AND_11_FOR_PROPER_DIRECTION = Units.inchesToMeters(30);

    public boolean add(Autos.AutoNotes note) {
        var isNote3 = note == Autos.AutoNotes.BlueNote3;
        var isNote11 = note == Autos.AutoNotes.RedNote11;

        var n = note.getNotePosition();
        var s = note.getShootPostion();
        var steal = note.getShootOrStealNote().equals(ShooterType.Steal);

        /**
         * Add a shoot point before note 3 and note 11
         * so that the robot picks up the note without hitting the stage
         */
        if ((isNote3 || isNote11) && !isEmpty()) {
            var prev = peekLast();
            var shiftMultipler = isNote3 ? -1 : 1;
            var workaroundPosition = n.getPosition().plus(
                    new Translation2d(shiftMultipler * SHIFT_FOR_3_AND_11_FOR_PROPER_DIRECTION, 0));
            if (prev.getType().equals(AutoPositionType.Shoot))
                removeLast();
            add(new AutoPosition(
                    workaroundPosition,
                    AutoPositionType.Shoot, prev.getShootOrStealNote()).withNotSkippable());
        }

        add(n);
        if (s != null && !steal) {
            add(s);
        }
        return true;
    }

}
