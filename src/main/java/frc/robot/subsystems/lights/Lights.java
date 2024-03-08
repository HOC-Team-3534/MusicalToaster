package frc.robot.subsystems.lights;

import java.util.Optional;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    static Spark blinkin = new Spark(0);

    private static final boolean enabled = true;
    private static Lights INSTANCE;

    public static Optional<Lights> createInstance() {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.empty();
        INSTANCE = new Lights();
        blinkin.set(LightModes.Default.value);
        return Optional.of(INSTANCE);
    }

    public static Optional<Lights> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    public enum LightModes {
        Default(0),
        StrobeRed(-0.11),
        StrobeBlue(-0.09),
        StrobeGold(-0.07),
        SolidGreen(0.75),
        SolidYellow(0.67);

        double value;

        LightModes(double value) {
            this.value = value;
        }
    }

    public Command applyLightMode(LightModes mode) {
        return startEnd(() -> blinkin.set(mode.value), () -> blinkin.set(LightModes.Default.value));
    }

}
