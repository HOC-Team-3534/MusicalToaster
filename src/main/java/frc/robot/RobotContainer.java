// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.concurrent.Callable;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive.FIELD_DIMENSIONS;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.commands.AutoPositionList;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRequest.ControlIntake;
import frc.robot.subsystems.swervedrive.FieldCentricWithProperDeadband;
import frc.robot.subsystems.turret.ShooterRequest.ControlShooter;
import frc.robot.subsystems.turret.ShooterRequest;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequest;
import frc.robot.subsystems.turret.TurretRequest.AimForAmp;
import frc.robot.subsystems.turret.TurretRequest.AimForSpeaker;
import frc.robot.subsystems.turret.TurretRequest.CalibrateShooter;
import frc.robot.subsystems.turret.TurretRequest.IndexFromIntake;
import frc.robot.subsystems.turret.TurretRequest.ShootFromSubwoofer;
import frc.robot.subsystems.turret.TurretRequest.TestingTurret;
import frc.robot.utils.PositionUtils;
import swerve.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// The driver station connected controllers are defined here...
	private double MaxAngularRate = 4 * Math.PI;

	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
	private final FieldCentricWithProperDeadband drive = new FieldCentricWithProperDeadband()
			.withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
			.withMaxSpeed(TunerConstants.kSpeedAt12VoltsMps).withMaxAngularSpeed(MaxAngularRate)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

	private final static Intake intake = new Intake();
	private final ControlIntake runIntake = new ControlIntake().withIntakePercent(0.5);
	private final ControlIntake runExtake = new ControlIntake().withIntakePercent(0.5).withIntakeReversed(true);
	private final ControlIntake stopIntake = new ControlIntake().withIntakePercent(0.0);
	private final ControlShooter shooterOff = new ControlShooter().withVelocity(0);
	private final static ControlShooter shooterAmp = new ControlShooter().withVelocity(1000);// TODO Find these valuess
	private final static ControlShooter shooterSpeaker = new ControlShooter().withVelocity(6200);// TODO Find these

	private final static Turret turret = new Turret(() -> drivetrain.getState(), () -> drivetrain.getChassisSpeeds());
	private final static IndexFromIntake indexFromIntake = new IndexFromIntake().withRollerOutput(0.25)
			.withRotateTolerance(Rotation2d.fromDegrees(1)).withTilt(Rotation2d.fromDegrees(-35))
			.withIntakeState(() -> intake.getState());// TODO Validate this

	private final TestingTurret testingShooter = new TestingTurret();
	private final CalibrateShooter calibrateShooter = new CalibrateShooter().withRollerOutput(0.25);

	private static SendableChooser<Autos.AutoNotes>[] noteHiearchyChoosers;
	private static SendableChooser<Autos.ShootOrStealNote>[] shootOrStealChoosers;

	// Tune
	// all,
	// values
	private final static AimForSpeaker aimForSpeaker = new AimForSpeaker().withRollerOutput(0.25)
			.withRotateTolerance(Rotation2d.fromDegrees(1))
			.withTiltTolerance(Rotation2d.fromDegrees(1)).withTiltFunction((distance) -> new Rotation2d())
			.withSwerveDriveState(() -> drivetrain.getState());// TODO Find distance table
	private final static AimForAmp aimForAmp = new AimForAmp().withRollerOutput(0.25)
			.withRotateTolerance(Rotation2d.fromDegrees(1))
			.withTiltTolerance(Rotation2d.fromDegrees(1)).withTilt(Rotation2d.fromDegrees(10))
			.withSwerveDriveState(() -> drivetrain.getState());// TODO Find the actualy tilt for aiming for amp
	private final ShootFromSubwoofer shootFromSubwoofer = new ShootFromSubwoofer().withRollerPercent(0.25)
			.withRotation(new Rotation2d()).withTilt(Rotation2d.fromDegrees(35))
			.withTolerance(Rotation2d.fromDegrees(1));

	private final boolean CHARACTERIZATION_ENABLED = true;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// Set Default Commands for Subsystems
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() -> drive
								.withVelocityX(
										// Drive forward with negative Y (forward)
										-driverController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps
												* getCoordinateSystemInversionDriving())
								.withVelocityY(
										// Drive left with negative X (left)
										-driverController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps
												* getCoordinateSystemInversionDriving())
								.withRotationalRate(
										// Drive counterclockwise with negative X (left)
										-driverController.getRightX() * MaxAngularRate)
								.withCreepEnabled(driverController.getRightTriggerAxis() > 0.15)));

		intake.setDefaultCommand(intake.applyRequest(() -> stopIntake));

		if (!EnabledDebugModes.testingTurret)
			turret.setDefaultCommand(
					turret.applyRequest(() -> indexFromIntake, () -> shooterOff));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}

		drivetrain.registerTelemetry(logger::telemeterize);

		// Show Status of Subsystems on Dashboard
		for (int i = 0; i < 5; i++) {
			noteHiearchyChoosers[i] = newNoteHiearchyChooser();
			SmartDashboard.putData("Note Hiearchy " + (i + 1), noteHiearchyChoosers[i]);
			shootOrStealChoosers[i] = newShootOrStealChooser();
			SmartDashboard.putData("Shoot|Steal " + (i + 1), shootOrStealChoosers[i]);
		}
	}

	public static SendableChooser<Autos.AutoNotes> newNoteHiearchyChooser() {
		SendableChooser<Autos.AutoNotes> noteHiearchy = new SendableChooser<>();

		noteHiearchy.setDefaultOption("None", null);
		noteHiearchy.addOption("Blue Note 1", Autos.AutoNotes.BlueNote1);
		noteHiearchy.addOption("Blue Note 2", Autos.AutoNotes.BlueNote2);
		noteHiearchy.addOption("Blue Note 3", Autos.AutoNotes.BlueNote3);
		noteHiearchy.addOption("Center Note 4", Autos.AutoNotes.MiddleNote4);
		noteHiearchy.addOption("Center Note 5", Autos.AutoNotes.MiddleNote5);
		noteHiearchy.addOption("Center Note 6", Autos.AutoNotes.MiddleNote6);
		noteHiearchy.addOption("Center Note 7", Autos.AutoNotes.MiddleNote7);
		noteHiearchy.addOption("Center Note 8", Autos.AutoNotes.MiddleNote8);
		noteHiearchy.addOption("Red Note 9", Autos.AutoNotes.RedNote9);
		noteHiearchy.addOption("Red Note 10", Autos.AutoNotes.RedNote10);
		noteHiearchy.addOption("Red Note 11", Autos.AutoNotes.RedNote11);

		return noteHiearchy;
	}

	public static SendableChooser<Autos.ShootOrStealNote> newShootOrStealChooser() {
		SendableChooser<Autos.ShootOrStealNote> shootOrSteal = new SendableChooser<>();

		shootOrSteal.setDefaultOption("Shoot", Autos.ShootOrStealNote.Shoot);
		shootOrSteal.addOption("Steal", Autos.ShootOrStealNote.Steal);

		return shootOrSteal;
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
	 * with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
	 * subclasses for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		driverController.a().and(() -> CHARACTERIZATION_ENABLED).whileTrue(drivetrain.characterizeDrive(1.0, 4.0));
		driverController.b().whileTrue(drivetrain
				.applyRequest(
						() -> point.withModuleDirection(
								new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

		// reset the field-centric heading on left bumper press
		TGR.ResetFieldRelative.tgr().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		TGR.Intake.tgr().whileTrue(intake.applyRequest(() -> runIntake));

		TGR.ShootSpeaker.tgr().whileTrue(getShootCommand(aimForSpeaker, shooterSpeaker));
		TGR.ShootAmp.tgr().whileTrue(getShootCommand(aimForAmp, shooterAmp));
		TGR.ShootFromSubwoofer.tgr().whileTrue(turret.applyRequest(
				() -> shootFromSubwoofer.withReadyToShoot(() -> TGR.PrepareShootForSubwoofer.tgr().getAsBoolean()),
				() -> shooterSpeaker));

		TGR.TestingRotationPositive.tgr()
				.whileTrue(turret.applyRequest(() -> testingShooter.withPercentRotate(0.05).withPercentTilt(0),
						() -> shooterOff));
		TGR.TestingRotationNegative.tgr()
				.whileTrue(turret.applyRequest(() -> testingShooter.withPercentRotate(-0.05).withPercentTilt(0),
						() -> shooterOff));
		TGR.TestingTiltPositive.tgr()
				.whileTrue(turret.applyRequest(() -> testingShooter.withPercentRotate(0).withPercentTilt(0.05),
						() -> shooterOff));
		TGR.TestingTiltNegative.tgr()
				.whileTrue(turret.applyRequest(() -> testingShooter.withPercentRotate(0).withPercentTilt(-0.05),
						() -> shooterOff));

	}

	public static AutoPositionList getAutoPositions() {
		AutoPositionList positions = new AutoPositionList();
		for (int i = 0; i < noteHiearchyChoosers.length && i < shootOrStealChoosers.length; i++) {
			var note = noteHiearchyChoosers[i].getSelected();
			var shootOrSteal = shootOrStealChoosers[i].getSelected();
			if (note != null)
				positions.add(note.withShootOrStealNote(shootOrSteal));
		}
		return positions;
	}

	public static int getCoordinateSystemInversionDriving() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() && alliance.get() == Alliance.Red ? -1 : 1;
	}

	public static Command getShootSpeakerCommand() {
		return getShootCommand(aimForSpeaker, shooterSpeaker);
	}

	private static Command getShootCommand(TurretRequest aimForRequest, ShooterRequest shooterRequest) {
		return Commands.either(
				turret.applyRequest(() -> indexFromIntake, () -> shooterRequest),
				turret.applyRequest(() -> aimForRequest, () -> shooterRequest),
				() -> !turret.getState().noteLoaded || !isValidShootPosition());
	}

	public static boolean isValidShootPosition() {
		var x = drivetrain.getState().Pose.getX();
		var alliance = DriverStation.getAlliance().get();
		var behindAllianceLine = alliance.equals(Alliance.Blue)
				? x < FIELD_DIMENSIONS.CENTER_OF_FIELD.minus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER).getX()
				: x > FIELD_DIMENSIONS.CENTER_OF_FIELD.plus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER).getX();
		if (!behindAllianceLine)
			return false;
		return !isRobotUnderStage();
	}

	public static Translation2d STAGE_BOUNDARY_1 = FIELD_DIMENSIONS.CENTER_OF_FIELD
			.minus(FIELD_DIMENSIONS.OFFSET_CENTER_TO_SIDE_ROW_OF_NOTES);
	public static Translation2d BETWEEN_BOUNDARY_2_AND_3 = FIELD_DIMENSIONS.CENTER_OF_FIELD
			.minus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER);
	public static Translation2d STAGE_BOUNDARY_2 = BETWEEN_BOUNDARY_2_AND_3.plus(FIELD_DIMENSIONS.OFFSET_CENTER_NOTES);
	public static Translation2d STAGE_BOUNDARY_3 = BETWEEN_BOUNDARY_2_AND_3.minus(FIELD_DIMENSIONS.OFFSET_CENTER_NOTES);

	public static boolean isRobotUnderStage() {
		var current = drivetrain.getState().Pose.getTranslation();
		var curretFlippedForBlue = current.getX() > FIELD_DIMENSIONS.CENTER_OF_FIELD.getX()
				? new Translation2d(FIELD_DIMENSIONS.LENGTH - current.getX(), current.getY())
				: current;
		return PositionUtils.isPointInTriangle(curretFlippedForBlue, STAGE_BOUNDARY_1, STAGE_BOUNDARY_2,
				STAGE_BOUNDARY_3);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		var positions = getAutoPositions();
		if (positions.isEmpty()) {
			// return Shoot and Drive Forward Command;
		}
		return Autos.getDynamicAutonomous(turret, drivetrain, positions);
	}

	public enum TGR {
		Creep(driverController.leftBumper()),
		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled)),
		ResetFieldRelative(driverController.start()),
		Intake(driverController.rightTrigger(0.15)),
		Extake(driverController.rightBumper()),
		ShootSpeaker(driverController.x().and(() -> !EnabledDebugModes.testingTurret)),
		ShootAmp(driverController.b().and(() -> !EnabledDebugModes.testingTurret)),
		PrepareShootForSubwoofer(operatorController.a().and(() -> !EnabledDebugModes.testingTurret)),
		ShootFromSubwoofer(operatorController.rightTrigger().and(() -> !EnabledDebugModes.testingTurret)),
		CalibrateShooter(driverController.b()
				.and(() -> !EnabledDebugModes.testingTurret && EnabledDebugModes.calibratingTurret)),
		TestingRotationPositive(driverController.rightTrigger(0.15).and(() -> EnabledDebugModes.testingTurret)),
		TestingRotationNegative(driverController.leftTrigger(0.15).and(() -> EnabledDebugModes.testingTurret)),
		TestingTiltPositive(driverController.rightBumper().and(() -> EnabledDebugModes.testingTurret)),
		TestingTiltNegative(driverController.leftBumper().and(() -> EnabledDebugModes.testingTurret));

		Trigger trigger;

		TGR(Trigger trigger) {
			this.trigger = trigger;
		}

		public Trigger tgr() {
			return trigger.and(() -> !Robot.isAutonomous);
		}

		public boolean bool() {
			return trigger.getAsBoolean();
		}
	}

	public enum AXS {
		Drive_ForwardBackward(() -> slewRateLimiterX.calculate(-modifyAxis(driverController.getLeftY()))),
		Drive_LeftRight(() -> slewRateLimiterY.calculate(-modifyAxis(driverController.getLeftX()))),
		Drive_Rotation(() -> slewRateLimiterRotation.calculate(-modifyAxis(driverController.getRightX())));

		Callable<Double> callable;

		AXS(Callable<Double> callable) {
			this.callable = callable;
		}

		public double getAxis() {
			try {
				return callable.call().doubleValue();
			} catch (Exception ex) {
				return 0.0;
			}
		}
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);
		// Square the axis
		value = Math.copySign(value * value, value);
		return value;
	}
}
