package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Drive.AUTO;
import frc.robot.Constants.Drive.Config;
import frc.robot.Constants.RobotType;
import frc.robot.Path;
import frc.robot.RobotContainer.AXS;
import frc.robot.RobotContainer.TGR;
import frc.robot.SwerveHelper;
import swerve.SwerveDrivetrainModel;
import swerve.SwerveInput;
import swerve.SwerveModule;
import swerve.SwerveSubsystem;
import swerve.params.SwerveParams;

public class SwerveDrive extends SwerveSubsystem {
    // TODO determine CBOT angle offsets
    final static double fl_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 86.13
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 273.07 : 134.56;
    final static double fr_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 3.86
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 304.62 : 36.035;
    final static double bl_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 274.30
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 130.86 : 174.19;
    final static double br_degrees = (Constants.ROBOTTYPE == RobotType.TBOT) ? 23.90
            : (Constants.ROBOTTYPE == RobotType.PBOT) ? 20.39 : 153.37;
    final static SwerveParams swerveParams = SwerveHelper.loadSwerveParams();
    final static TalonFX FL_drive = new TalonFX(1);
    final static TalonFX FL_steer = new TalonFX(3);
    final static CANcoder FL_cancoder = new CANcoder(2);
    final static SwerveModule fl = new SwerveModule(FL_drive, FL_steer, FL_cancoder,
            Rotation2d.fromDegrees(fl_degrees));
    final static TalonFX FR_drive = new TalonFX(4);
    final static TalonFX FR_steer = new TalonFX(6);
    final static CANcoder FR_cancoder = new CANcoder(5);
    final static SwerveModule fr = new SwerveModule(FR_drive, FR_steer, FR_cancoder,
            Rotation2d.fromDegrees(fr_degrees));
    final static TalonFX BL_drive = new TalonFX(7);
    final static TalonFX BL_steer = new TalonFX(9);
    final static CANcoder BL_cancoder = new CANcoder(8);
    final static SwerveModule bl = new SwerveModule(BL_drive, BL_steer, BL_cancoder,
            Rotation2d.fromDegrees(bl_degrees));
    final static TalonFX BR_drive = new TalonFX(10);
    final static TalonFX BR_steer = new TalonFX(12);
    final static CANcoder BR_cancoder = new CANcoder(11);
    final static SwerveModule br = new SwerveModule(BR_drive, BR_steer, BR_cancoder,
            Rotation2d.fromDegrees(br_degrees));
    final static Pigeon2 pigeon2 = new Pigeon2(Config.PIGEON2_ID);
    static SwerveDrivetrainModel dt;
    Pose2d gridPose;
    double timeCharacterizing;
    final Field2d field = new Field2d();

    public SwerveDrive() {
        super(dt);

        dt = new SwerveDrivetrainModel(fl, fr, bl, br, pigeon2, swerveParams, this);

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        super.periodic();
        field.setRobotPose(dt.getPose());
        SmartDashboard.putNumber("Pitch", getSlope());

    }

    public SwerveDrivetrainModel getDT() {
        return dt;
    }

    public Command brake() {
        return Commands.runOnce(this::setAllBrakeMode);
    }

    private void setAllBrakeMode() {
        setNeutralMode(FL_drive, NeutralModeValue.Brake);
        setNeutralMode(FR_drive, NeutralModeValue.Brake);
        setNeutralMode(BL_drive, NeutralModeValue.Brake);
        setNeutralMode(BR_drive, NeutralModeValue.Brake);
    }

    public Command coast() {
        return Commands.runOnce(this::setAllCoastMode);
    }

    private void setAllCoastMode() {
        setNeutralMode(FL_drive, NeutralModeValue.Coast);
        setNeutralMode(FR_drive, NeutralModeValue.Coast);
        setNeutralMode(BL_drive, NeutralModeValue.Coast);
        setNeutralMode(BR_drive, NeutralModeValue.Coast);
    }

    private static void setNeutralMode(TalonFX motor, NeutralModeValue mode) {
        var motorOutputConfigs = new MotorOutputConfigs();

        motor.getConfigurator().refresh(motorOutputConfigs);
        motorOutputConfigs.withNeutralMode(mode);
        motor.getConfigurator().apply(motorOutputConfigs);
    }

    public void updatePoseWithVision(Pose2d pose, double latency) {
        dt.updateOdometryWithVision(pose, latency, false);
    }

    public double getSlope() {
        return dt.getSlope(((RobotType.PBOT == Constants.ROBOTTYPE) ? -3.0 : -0.5));
    }

    public boolean isFacingForward() {
        return dt.isFacingForward(2.0);
    }

    public void setKnownPose(Path path) {
        dt.setKnownPose(path.getPath().getPreviewStartingHolonomicPose());
    }

    public Command followPath(Path path) {
        return dt.followPath(path.getPath());
    }

    /**
     * Drives the robot using forward backward and left right and rotation inputs.
     * Has Creep mode to move the robot at a lower speed
     * 
     * @return The command that moves the robot
     */
    public Command drive() {
        var command = run(() -> {
            dt.setModuleStates(new SwerveInput(AXS.Drive_ForwardBackward.getAxis(),
                    AXS.Drive_LeftRight.getAxis(),
                    AXS.Drive_Rotation.getAxis()), TGR.Creep.bool(), false);
        });

        command.setName("Drive Manually");
        return command;
    }

    /**
     * Follows an autonomously generated path to a specific grid position in which
     * the operator chooses generates a path on the fly and drives the robot in DTM
     * to the position
     * 
     * @param gridPose The grid Position the robot will generate a path to(Left
     *                 Right or Center)
     * @return The command that the robot will use to autonomously follow in DTM to
     *         a grid position
     */

    public Command followPathtoGridPose(Pose2d gridPose) {
        this.gridPose = gridPose;
        if (gridPose == null)
            return Commands.print("Grid Pose Null");
        return dt.pathfindToPose(gridPose, AUTO.kPathConstraints, 0.0);
    }

    /**
     * Checks to see if there is a valid grid position that limelight picks up
     * 
     * @return Valid grid position
     */
    public boolean isGridPoseValid() {
        return gridPose != null;
    }

    public Pose2d getPose() {
        return dt.getPose();
    }
}
