package frc.robot.utils.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.utils.EdgeDetectionUtils;

public class SendableChooserWithChangeDetector<T> extends SendableChooser<T> {

    Optional<T> lastValue = Optional.empty();

    public SendableChooserWithChangeDetector() {
        super();
    }

    public Optional<T> detectChange() {
        var selected = Optional.ofNullable(getSelected());
        var changed = EdgeDetectionUtils.detectChange(lastValue, selected);
        lastValue = selected;

        if (changed)
            return selected;
        else
            return Optional.empty();
    }

}
