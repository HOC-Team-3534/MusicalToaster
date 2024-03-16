package frc.robot.utils.sensors;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;

public class QuadEncoderOnSRX {

    final TalonSRX talonSRX;

    static double TICKS_PER_ROTATION = 1440;

    public QuadEncoderOnSRX(TalonSRX talonSRX) {
        this.talonSRX = talonSRX;
        this.talonSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    public QuadEncoderOnSRX withSensorPhase(boolean sensorPhase) {
        this.talonSRX.setSensorPhase(sensorPhase);
        return this;
    }

    public void setRotation(Rotation2d rotation) {
        this.talonSRX.setSelectedSensorPosition(rotation.getRotations() * TICKS_PER_ROTATION);
    }

    public Rotation2d getRotation() {
        return Rotation2d
                .fromRotations(this.talonSRX.getSelectedSensorPosition() / TICKS_PER_ROTATION);
    }

}
