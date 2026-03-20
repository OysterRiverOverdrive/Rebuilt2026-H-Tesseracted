// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera1;
  PhotonCamera camera2;
  PhotonCamera camera3;
  PhotonCamera camera4;

  ArrayList<PhotonCamera> cameras = new ArrayList<>();

  PhotonPoseEstimator photonEstimatorCam1;
  PhotonPoseEstimator photonEstimatorCam2;
  PhotonPoseEstimator photonEstimatorCam3;
  PhotonPoseEstimator photonEstimatorCam4;

  ArrayList<PhotonPoseEstimator> photonEstimators = new ArrayList<>();

  Field2d[] poseEstField = new Field2d[4];
  EstimateConsumer[] estCons = new EstimateConsumer[4];

  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  AprilTagFieldLayout fieldmap;

  public VisionSubsystem() {
    camera1 = new PhotonCamera("Camera1");
    camera2 = new PhotonCamera("Camera2");
    camera3 = new PhotonCamera("Camera3");
    camera4 = new PhotonCamera("Camera4");

    // Add other cameras once they are added to robot
    cameras.add(camera1);
    cameras.add(camera2);
    cameras.add(camera3);
    cameras.add(camera4);

    // initialize fields for pose estimate display/logging
    for (int i = 0; i < 4; i++) {
      poseEstField[i] = new Field2d();
      SmartDashboard.putData("Camera " + (i + 1), poseEstField[i]);

      estCons[i] = new EstimateConsumer();
    }

    fieldmap = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    photonEstimatorCam1 = new PhotonPoseEstimator(fieldmap, VisionConstants.kRobotToCam1);

    photonEstimatorCam2 = new PhotonPoseEstimator(fieldmap, VisionConstants.kRobotToCam2);

    photonEstimatorCam3 = new PhotonPoseEstimator(fieldmap, VisionConstants.kRobotToCam3);

    photonEstimatorCam4 = new PhotonPoseEstimator(fieldmap, VisionConstants.kRobotToCam4);

    // Add other photonEstimators when other cameras are added
    photonEstimators.add(photonEstimatorCam1);
    photonEstimators.add(photonEstimatorCam2);
    photonEstimators.add(photonEstimatorCam3);
    photonEstimators.add(photonEstimatorCam4);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(fieldmap);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      cameraSim = new PhotonCameraSim(camera1, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(cameraSim, VisionConstants.kRobotToCam1);

      cameraSim.enableDrawWireframe(true);
    }
  }

  public void periodic() {
    boolean staleVision = true;
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (int i = 0; i < 4; i++) {
      PhotonCamera camera = cameras.get(i);
      for (var change : camera.getAllUnreadResults()) {
        visionEst = photonEstimators.get(i).estimateCoprocMultiTagPose(change);
        if (visionEst.isEmpty()) {
          visionEst = photonEstimators.get(i).estimateLowestAmbiguityPose(change);
        }
        updateEstimationStdDevs(visionEst, change.getTargets());

        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
              est ->
                  getSimDebugField()
                      .getObject("VisionEstimation")
                      .setPose(est.estimatedPose.toPose2d()),
              () -> {
                getSimDebugField().getObject("VisionEstimation").setPoses();
              });
        }

        // the index variable needs to be final to use in ifPresent
        final int fi = i;
        visionEst.ifPresent(
            est -> {
              poseEstField[fi].setRobotPose(est.estimatedPose.toPose2d());
              // Change our trust in the measurement based on the tags we can see
              var estStdDevs = getEstimationStdDevs();

              estCons[fi].accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
        if (visionEst.isPresent() == false) {
          estCons[i].setStale();
        }
      }

      staleVision = staleVision && estCons[i].isStale();
    }

    SmartDashboard.putBoolean("Stale Vision", staleVision);
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        for (int i = 0; i < 4; i++) {
          var tagPose = photonEstimators.get(i).getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
