package frc.robot.utils;

import java.util.Optional;

public class EdgeDetectionUtils {
    public static <T> boolean detectChange(Optional<T> current, Optional<T> new_) {
        return (current.isEmpty() && new_.isPresent())
                || (current.isPresent() && new_.isEmpty())
                || (current.isPresent() && new_.isPresent()
                        && !current.get().equals(new_.get()));
    }
}
