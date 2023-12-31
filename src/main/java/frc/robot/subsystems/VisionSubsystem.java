// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final double CAMERA_HEIGHT_METERS = .0698;
  private static final double TARGET_HEIGHT_METERS = .279;
  private static final double CAMERA_PITCH_RADIANS = 0.0;
  PhotonCamera m_camera = new PhotonCamera("OV9281");
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var result = m_camera.getLatestResult();
    // System.out.println("Camera results:"+ result.toString());
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      int targetNumber = target.getFiducialId();
      System.out.println(" Target number:"+targetNumber+" Yaw, pitch:" + target.getYaw()+ "  " + target.getPitch());
    }
    getRange();
  }

  /**
   * calculate the range to the best target
   */
  private void getRange(){
    var result = m_camera.getLatestResult();

    if (result.hasTargets()) {
        // First calculate range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));
        System.out.println(" Range to target:"+range);
    }
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /**
   * Get a target (if any) for the given ID
   * 
   */
  public PhotonTrackedTarget getTargetForTag(int AprilTagFiducialID){
    PhotonTrackedTarget target = null;
    var photonRes = m_camera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
           
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == AprilTagFiducialID)
          .findFirst();
      if (targetOpt.isPresent()) {
        target = targetOpt.get();
      }
      
    }
    return target;
  }
}
