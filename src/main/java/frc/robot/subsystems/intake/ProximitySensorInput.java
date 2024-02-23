package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;

public class ProximitySensorInput extends DigitalInput {

    public ProximitySensorInput(int channel) {
        super(channel);
    }

    @Override
    public boolean get() {
        return !super.get();
    }

}
