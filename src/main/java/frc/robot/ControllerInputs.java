package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.utils.ControllerUtils;

public class ControllerInputs {
    private static final CommandXboxController driverController = new CommandXboxController(0);
    private static final CommandXboxController operatorController = new CommandXboxController(1);

    public enum BTN {
        TiltFlat(() -> driverController.getHID().getLeftBumper() || operatorController.getHID().getBButton()),
        Creep(() -> driverController.getHID().getLeftTriggerAxis() > 0.15),
        SubwooferLetItRip(() -> operatorController.getHID().getRightBumper()),
        ReloadNoteActivate(() -> SubwooferLetItRip.get()),
        ResetClimber(() -> driverController.getHID().getPOV() == 180 && TiltFlat.get());

        Supplier<Boolean> buttonSupplier;

        BTN(Supplier<Boolean> buttonSupplier) {
            this.buttonSupplier = buttonSupplier;
        }

        public boolean get() {
            return buttonSupplier.get();
        }
    }

    public enum TGR {
        Intake(driverController.rightTrigger(0.15), false),
        ShootSpeaker(operatorController.rightTrigger(0.15), true),
        Extake(driverController.rightBumper(), false),
        PrepareShootForSubwoofer(operatorController.x(), true),
        Climb(operatorController.start().and(() -> driverController.getHID().getStartButton()), false),
        ReloadNote(operatorController.back(), true),
        AmpLights(operatorController.y(), false),

        ResetNoteinRobot(driverController.back(), false),
        // Below are debugging actions
        ToggleAutoAimSubwoofer(operatorController.rightStick(), false),

        Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled), false),
        ResetClimber(new Trigger(() -> BTN.ResetClimber.get()).and(() -> !DriverStation.isFMSAttached()), false);

        Trigger trigger;

        TGR(Supplier<Boolean> booleanSupplier, boolean blockedByClimbing) {
            this(new Trigger(() -> booleanSupplier.get()), blockedByClimbing);
        }

        TGR(Trigger trigger, boolean blockedByClimbing) {
            this.trigger = blockedByClimbing ? trigger.and(() -> !RobotContainer.getRobotState().isClimbing())
                    : trigger;
        }

        public Trigger tgr() {
            return trigger.and(() -> !Robot.isAutonomous);
        }
    }

    public enum AXS {
        Drive_ForwardBackward(() -> driverController.getLeftY(), true),
        Drive_LeftRight(() -> driverController.getLeftX(), true),
        Drive_Rotation(() -> driverController.getRightX());

        final Supplier<Double> supplier;
        Optional<SlewRateLimiter> slewRateLimiter = Optional.empty();
        boolean allianceInvert;

        AXS(Supplier<Double> supplier, double rate, boolean allianceInvert) {
            this(supplier, rate);
            this.allianceInvert = allianceInvert;
        }

        AXS(Supplier<Double> supplier, boolean allianceInvert) {
            this(supplier);
            this.allianceInvert = allianceInvert;
        }

        AXS(Supplier<Double> supplier, double rate) {
            this(supplier);
            this.slewRateLimiter = Optional.of(new SlewRateLimiter(rate));
        }

        AXS(Supplier<Double> supplier) {
            this.supplier = () -> ControllerUtils.modifyAxis(-supplier.get(), getDeadband(), getCreepScale())
                    * (allianceInvert ? getAllianceInversion() : 1);
        }

        public double getAxis() {
            return applyRateLimit(supplier.get());
        }

        private double applyRateLimit(double value) {
            return slewRateLimiter.map(limiter -> limiter.calculate(value)).orElse(value);
        }

        public static int getAllianceInversion() {
            return DriverStation.getAlliance().map(alliance -> alliance.equals(Alliance.Red) ? -1 : 1).orElse(1);
        }

        public static double getCreepScale() {
            return BTN.Creep.get() ? 0.25 : 1;
        }

        public static double getDeadband() {
            return 0.1;
        }
    }
}
