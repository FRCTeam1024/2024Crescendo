package frc.robot;

import static frc.robot.Constants.CameraConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVision implements Logged {
  private List<VisionCamera> cameras = new ArrayList<>();

  public AprilTagVision() {
    addCamera("FrontCamera", CameraConstants.frontCameraName, CameraConstants.frontCamTransform);
    addCamera("RearCamera", CameraConstants.rearCameraName, CameraConstants.rearCamTransform);
  }

  public void addCamera(String name, String photonName, Transform3d robotToCamera) {
    var camera = new PhotonCamera(photonName);
    var estimator =
        new PhotonPoseEstimator(
            Constants.kOfficialField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
    cameras.add(new VisionCamera(name, camera, estimator));
    StatusDashboard.addStatusIndicator(name + "Connected", camera::isConnected);
  }

  public void processVisionUpdates(Consumer<EstimatedRobotPose> poseConsumer, Pose2d curPose) {
    for (int i = 0; i < cameras.size(); i++) {
      var camera = cameras.get(i);
      var result = camera.camera().getLatestResult();
      var estimatedPose = camera.estimator().update(result);
      if (estimatedPose.isPresent() && shouldAcceptUpdate(result, estimatedPose.get(), curPose)) {
        log(camera.name() + "EstimatedPose", estimatedPose.get().estimatedPose);
        poseConsumer.accept(estimatedPose.get());
      }
    }
  }

  public boolean shouldAcceptUpdate(
      PhotonPipelineResult result, EstimatedRobotPose estimatedRobotPose, Pose2d curPose) {

    // Reject if off field
    if (!translationWithinField(estimatedRobotPose.estimatedPose.toPose2d().getTranslation())) {
      return false;
    }

    // Reject if > 6" above or below the ground
    if (Math.abs(estimatedRobotPose.estimatedPose.getZ()) > Units.inchesToMeters(6)) {
      return false;
    }

    // Always accept if we have a multitag result and the size is large enough
    if (result.getMultiTagResult().estimatedPose.isPresent
        && estimatedRobotPose.targetsUsed.get(0).getArea() > 0.05) {
      return true;
    }

    var apriltagPose =
        Constants.kOfficialField.getTagPose(estimatedRobotPose.targetsUsed.get(0).getFiducialId());
    if (apriltagPose.isEmpty()) {
      return false;
    }

    // Reject if tag is too far away
    var distanceToTag =
        curPose.getTranslation().getDistance(apriltagPose.get().getTranslation().toTranslation2d());
    if (distanceToTag > 7) {
      return false;
    }

    return true;
  }

  private boolean translationWithinBounds(
      Translation2d value, Translation2d min, Translation2d max) {
    return (value.getX() > min.getX() && value.getX() < max.getX())
        && (value.getY() > min.getY() && value.getY() < max.getY());
  }

  private boolean translationWithinField(Translation2d val) {
    return translationWithinBounds(
        val,
        new Translation2d(),
        new Translation2d(
            Constants.kOfficialField.getFieldLength(), Constants.kOfficialField.getFieldWidth()));
  }

  public static record VisionCamera(
      String name, PhotonCamera camera, PhotonPoseEstimator estimator) {}
}
