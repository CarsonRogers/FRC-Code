// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.sensors.PhotonCam;

/**
 * Represents the PhotonVision subsystem which handles vision processing and tracking
 * using Photon Cameras
 * 
 * For each camera on the robot, a new PhotonCamera will need created
 */
public class PhotonVision extends SubsystemBase {
  private final PhotonCam frontCamera;
  private final PhotonCam backCamera;

  private AprilTagFieldLayout fieldLayout;

  public PhotonVision(Swerve swerve) {
    try {
      fieldLayout = 
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e){
      e.printStackTrace();
    }

    frontCamera = new PhotonCam(
      VisionConstants.frontCamName, 
      VisionConstants.frontRobotToCam, 
      swerve, fieldLayout);

    backCamera = new PhotonCam(
      VisionConstants.backCamName, 
      VisionConstants.backRobotToCam, 
      swerve, fieldLayout);
  }

  /**
   * Retrieves the pose (position and orientation) of a specific tag 
   * identified by its ID
   * 
   * @param id The ID of the tag
   * @return An Optional containing the pose of the tag if it is found, or an empty
   *  Optional if the tag is not found
   */
  public Optional<Pose3d> getTagPose(int id){
    return fieldLayout.getTagPose(id);
  }

  /**
   * Updates all photon cameras to get their latest results and filter/add vision measurements
   */
  @Override
  public void periodic() {
    frontCamera.update();
    backCamera.update();
  }
}
