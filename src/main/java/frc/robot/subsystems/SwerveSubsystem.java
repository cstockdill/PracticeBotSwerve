package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetWheelAlignment;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FrontLeft.DriveMotor,
            DriveConstants.FrontLeft.TurningMotor,
            DriveConstants.FrontLeft.DriveEncoderReversed,
            DriveConstants.FrontLeft.TurningEncoderReversed,
            DriveConstants.FrontLeft.TurningAbsoluteEncoder,
            DriveConstants.FrontLeft.DriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FrontRight.DriveMotor,
            DriveConstants.FrontRight.TurningMotor,
            DriveConstants.FrontRight.DriveEncoderReversed,
            DriveConstants.FrontRight.TurningEncoderReversed,
            DriveConstants.FrontRight.TurningAbsoluteEncoder,
            DriveConstants.FrontRight.DriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BackLeft.DriveMotor,
            DriveConstants.BackLeft.TurningMotor,
            DriveConstants.BackLeft.DriveEncoderReversed,
            DriveConstants.BackLeft.TurningEncoderReversed,
            DriveConstants.BackLeft.TurningAbsoluteEncoder,
            DriveConstants.BackLeft.DriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BackRight.DriveMotor,
            DriveConstants.BackRight.TurningMotor,
            DriveConstants.BackRight.DriveEncoderReversed,
            DriveConstants.BackRight.TurningEncoderReversed,
            DriveConstants.BackRight.TurningAbsoluteEncoder,
            DriveConstants.BackRight.DriveAbsoluteEncoderReversed);

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());

    private double yTiltOffset;

    private MedianFilter yFilter = new MedianFilter(10);
    private double filtered_y;

    private GenericEntry turboSpeedFactor;
    private GenericEntry normalSpeedFactor;
    private GenericEntry dampenedSpeedFactor;


    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        loadPreferences();

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("Front Left", frontLeft)
            .withSize(2,2);
        swerveTab.add("Front Right", frontRight)
            .withSize(2,2);
        swerveTab.add("Back Right", backRight)
            .withSize(2,2)
            .withPosition(0, 2);
        swerveTab.add("Back Left", backLeft)
            .withSize(2,2)
            .withPosition(2, 2);

        swerveTab.add("Set Wheel Offsets", new SetWheelAlignment(this));
        
        turboSpeedFactor = swerveTab.add("Turbo Percentage", 0.9)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();

        normalSpeedFactor = swerveTab.add("Normal Percentage", .5)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();
 
        dampenedSpeedFactor = swerveTab.add("Dampened Percentage", .2)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();


            // Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            this // Reference to this subsystem to set requirements
        );
    }

    public double getTurboSpeedFactor(){
        return turboSpeedFactor.getDouble(0.5);
    }
    public double getNormalSpeedFactor(){
        return normalSpeedFactor.getDouble(0.3);
    }
    public double getDampenedSpeedFactor(){
        return dampenedSpeedFactor.getDouble(0.1);
    }

    public void lockEncoderOffset(){
        frontLeft.lockEncoderOffset();
        frontRight.lockEncoderOffset();
        backLeft.lockEncoderOffset();
        backRight.lockEncoderOffset();
    }

    public void loadPreferences(){
        frontLeft.loadPreferences();
        frontRight.loadPreferences();
        backLeft.loadPreferences();
        backRight.loadPreferences();
    }

    public void zeroHeading() {
        gyro.reset();
    }
    public void setHeading(double setAngle){
        //pigeon.setYaw(setAngle);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
    private void resetPose(Pose2d pose2d1) {
        resetOdometry(pose2d1);
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }
    public void resetOdometry(){
        resetOdometry(new Pose2d());
    }

    private SwerveModulePosition [] getModulePositions(){
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
            backRight.getPosition()};
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState [] statelist = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return statelist;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);

    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());

    }

    public void driveRobotRelative(ChassisSpeeds chassisspeeds1) {
         // 5. Convert chassis speeds to individual module states
         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisspeeds1);

         // 6. Output each module states to wheels
         setModuleStates(moduleStates);
    }

}