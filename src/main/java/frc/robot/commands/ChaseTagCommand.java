package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 1;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180.0));
   /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
  public static final Transform2d CAMERA_TO_ROBOT = 
        new Transform2d(new Translation2d(inchesToMeters(12.75), 0.0), new Rotation2d(0.0));

  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem drivetrainSubsystem;
  
  private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  private  DoublePublisher m_xPub;
  private  DoublePublisher m_yPub;
  private  DoublePublisher m_omegaPub;
  
  

  public ChaseTagCommand(
        VisionSubsystem visionSubsystem, 
        SwerveSubsystem drivetrainSubsystem) {
    this.m_VisionSubsystem = visionSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
   
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-1, 1);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    var robotPose = drivetrainSubsystem.getPose();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    setupPublishers();
  }

  private void setupPublishers(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Vision/goal");
    m_xPub = table.getDoubleTopic("X").publish();
    m_yPub = table.getDoubleTopic("Y").publish();
    m_omegaPub = table.getDoubleTopic("OmegaRad").publish();
  }

  @Override
  public void execute() {
      var robotPose = drivetrainSubsystem.getPose();
      var target = m_VisionSubsystem.getTargetForTag(TAG_TO_CHASE);

      if (target != null && !target.equals(lastTarget)) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          // Get the transformation from the camera to the tag (in 2d)
          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(
                  camToTarget.getTranslation().toTranslation2d(),
                  camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));  // not sure this 90 deg correctoin needed

          // Transform the robot's pose to find the tag's pose
          var cameraPose = robotPose.transformBy(CAMERA_TO_ROBOT.inverse());
          Pose2d targetPose = cameraPose.transformBy(transform);

          // Transform the tag's pose to set our goal
          goalPose = targetPose.transformBy(TAG_TO_GOAL);
      }

      if (goalPose != null) {
          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
          m_xPub.set(goalPose.getX());
          m_yPub.set(goalPose.getY());
          m_omegaPub.set(goalPose.getRotation().getRadians());

      }

      double xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
          xSpeed = 0;
      }

      double ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
          ySpeed = 0;
      }

      double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
          omegaSpeed = 0;
      }

      ChassisSpeeds goalSpeeds = new ChassisSpeeds(xSpeed,ySpeed,omegaSpeed);
      
      drivetrainSubsystem.driveRobotRelative(goalSpeeds);
      
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopModules();
  }

}