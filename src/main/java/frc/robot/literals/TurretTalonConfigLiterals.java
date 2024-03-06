package frc.robot.literals;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface TurretTalonConfigLiterals {

    public TalonFXConfiguration getRotateConfig();

    public TalonFXConfiguration getTiltConfig();

    public TalonFXConfiguration getShooterConfig();

    public class CBOT_TurretTalonConfigLiterals implements TurretTalonConfigLiterals {

        @Override
        public TalonFXConfiguration getRotateConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.75;
            cfg.MotionMagic.MotionMagicAcceleration = 0.35;
            cfg.MotionMagic.MotionMagicJerk = 20;

            cfg.Slot0.kP = 60;
            cfg.Slot0.kV = 13.906;
            cfg.Slot0.kS = 0.164;

            cfg.Feedback.SensorToMechanismRatio = 125;

            return cfg;
        }

        @Override
        public TalonFXConfiguration getTiltConfig() {
            return getTiltConfigWithPigeon();
        }

        private TalonFXConfiguration getTiltConfigWithPigeon() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.85;
            cfg.MotionMagic.MotionMagicAcceleration = 1.0;
            cfg.MotionMagic.MotionMagicJerk = 10;

            cfg.Slot0.kP = 200;
            cfg.Slot0.kV = 32.0713;
            cfg.Slot0.kS = 0.216898;

            cfg.Feedback.FeedbackRemoteSensorID = 22;
            cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemotePigeon2_Pitch;
            cfg.Feedback.SensorToMechanismRatio = 360.0;

            return cfg;
        }

        private TalonFXConfiguration getTiltConfigWithoutPigeon() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.85;
            cfg.MotionMagic.MotionMagicAcceleration = 1.0;
            cfg.MotionMagic.MotionMagicJerk = 10;

            cfg.Slot0.kP = 200;
            cfg.Slot0.kV = 32.0713;
            cfg.Slot0.kS = 0.216898;

            cfg.Feedback.SensorToMechanismRatio = 300;

            return cfg;
        }

        @Override
        public TalonFXConfiguration getShooterConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.Slot0.kP = 5;
            cfg.Slot0.kI = 0;
            cfg.Slot0.kA = 0.0115;
            cfg.Slot0.kV = 0.0230234;
            cfg.Slot0.kS = 1.5289;

            cfg.Feedback.SensorToMechanismRatio = 1;

            cfg.CurrentLimits.SupplyCurrentLimit = 30;

            return cfg;
        }

    }

    public class PBOT_TurretTalonConfigLiterals implements TurretTalonConfigLiterals {

        @Override
        public TalonFXConfiguration getRotateConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.75;
            cfg.MotionMagic.MotionMagicAcceleration = 0.15;
            cfg.MotionMagic.MotionMagicJerk = 20;

            cfg.Slot0.kP = 120;
            cfg.Slot0.kV = 13.29;
            cfg.Slot0.kS = 0.1763;
            cfg.Slot0.kD = 2.0;

            cfg.Feedback.SensorToMechanismRatio = 125;

            return cfg;
        }

        @Override
        public TalonFXConfiguration getTiltConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.85;
            cfg.MotionMagic.MotionMagicAcceleration = 1.0;
            cfg.MotionMagic.MotionMagicJerk = 10;

            cfg.Slot0.kP = 10;
            cfg.Slot0.kV = 32.55;
            cfg.Slot0.kS = 0.13;

            cfg.Feedback.SensorToMechanismRatio = 300;

            return cfg;
        }

        @Override
        public TalonFXConfiguration getShooterConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.Slot0.kP = 5;
            cfg.Slot0.kI = 0;
            cfg.Slot0.kA = 0.0115;
            cfg.Slot0.kV = 0.0230234;
            cfg.Slot0.kS = 1.5289;

            cfg.Feedback.SensorToMechanismRatio = 1;

            cfg.CurrentLimits.SupplyCurrentLimit = 30;

            return cfg;
        }

    }

    public class TBOT_TurretTalonConfigLiterals implements TurretTalonConfigLiterals {

        @Override
        public TalonFXConfiguration getRotateConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.75;
            cfg.MotionMagic.MotionMagicAcceleration = 0.15;
            cfg.MotionMagic.MotionMagicJerk = 20;

            cfg.Slot0.kP = 120;
            cfg.Slot0.kV = 13.29;
            cfg.Slot0.kS = 0.1763;
            cfg.Slot0.kD = 2.0;

            cfg.Feedback.SensorToMechanismRatio = 125;

            return cfg;
        }

        @Override
        public TalonFXConfiguration getTiltConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.MotionMagic.MotionMagicCruiseVelocity = 0.85;
            cfg.MotionMagic.MotionMagicAcceleration = 1.0;
            cfg.MotionMagic.MotionMagicJerk = 10;

            cfg.Slot0.kP = 10;
            cfg.Slot0.kV = 32.55;// TODO Tune these values
            cfg.Slot0.kS = 0.13;

            cfg.Feedback.SensorToMechanismRatio = 300;

            return cfg;
        }

        @Override
        public TalonFXConfiguration getShooterConfig() {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            cfg.Slot0.kP = 5;
            cfg.Slot0.kI = 0;
            cfg.Slot0.kA = 0.0115;
            cfg.Slot0.kV = 0.0230234;
            cfg.Slot0.kS = 1.5289;

            cfg.Feedback.SensorToMechanismRatio = 1;

            cfg.CurrentLimits.SupplyCurrentLimit = 30;

            return cfg;
        }

    }
}
