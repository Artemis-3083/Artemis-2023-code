// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class VisionSystem extends SubsystemBase {

//   PhotonCamera camera;
//   PhotonPipelineResult result;
//   PhotonTrackedTarget trackedResult;
//   Transform3d transform3d;
  
//   public VisionSystem() {
//     camera = new PhotonCamera("Microsoft_LifeCam_Studio(TM)");
//     camera.setDriverMode(false);
//     camera.setPipelineIndex(0); 
//   }

//   @Override
//   public void periodic() {
//     update();
//   }

//   public int getID(){
//     return camera.getLatestResult().getBestTarget().getFiducialId();
//   }

//   public void update(){
//     result = camera.getLatestResult();
//     if (result.hasTargets()) {
//       trackedResult = result.getBestTarget();
//       transform3d = trackedResult.getBestCameraToTarget();
//     }
//   }

//   public double getTagDistance(){
//     if (result.hasTargets()) {
//       return trackedResult.getBestCameraToTarget().getX();
//     }
//     return 0;
//   }

//   public double getTagHight(){
//     if (result.hasTargets()) {
//       return transform3d.getY();
//     }
//     return 0;
//   }

//   public double getTagAngle(){
//     if (result.hasTargets()) {
//       return transform3d.getZ();
//     }
//     return 0;
//   }
    

// }
