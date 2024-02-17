package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.IMU;
import frc.lib.hardware.Pigeon1IMU;
import frc.lib.hardware.Pigeon2IMU;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import java.util.List;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Swerve extends SubsystemBase implements Logged {
  private SwerveDrivePoseEstimator poseEstimator;
  private List<PhotonPoseEstimator> cameras;
  private SwerveModule[] mSwerveMods;
  private IMU gyro;
  private Field2d field = new Field2d();

  public Swerve() {
    SmartDashboard.putData(field);
    gyro = createGyro();
    gyro.setYaw(new Rotation2d());

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    AutoBuilder.configureHolonomic(
        this::getPose,
        (pose) -> {
          setPose(pose);
        },
        this::getRobotRelativeChassisSpeeds,
        this::driveRobotRelative,
        Constants.AutoConstants.pathFollowerConfig,
        () -> {
          var result = DriverStation.getAlliance();
          if (result.isEmpty()) {
            System.out.println("Alliance was empty at auto start!");
            return false;
          }
          return result.get().equals(Alliance.Red);
        },
        this);
    if (Constants.apriltagsEnabled) {
      cameras =
          List.of(
              new PhotonPoseEstimator(
                  Constants.kOfficialField,
                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                  new PhotonCamera("Arducam_OV9281_3"),
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(0.75),
                          Units.inchesToMeters(-11.25),
                          Units.inchesToMeters(27.625)),
                      new Rotation3d(
                          Units.degreesToRadians(0),
                          Units.degreesToRadians(-30),
                          Units.degreesToRadians(180)))));
    }
  }

  public IMU createGyro() {
    if (Constants.isPracticeBot) {
      var gyro = new PigeonIMU(Constants.Swerve.pigeonID);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 255);
      gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 255);
      return new Pigeon1IMU(gyro);
    } else {
      var gyro = new Pigeon2(Constants.Swerve.pigeonID);
      gyro.getConfigurator().apply(new Pigeon2Configuration());
      BaseStatusSignal.setUpdateFrequencyForAll(
          100, gyro.getYaw(), gyro.getPitch(), gyro.getRoll());
      if (Constants.disableUnusedSignals) {
        gyro.optimizeBusUtilization();
      }
      return new Pigeon2IMU(gyro);
    }
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    drive(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        fieldRelative,
        isOpenLoop);
  }

  public void drive(ChassisSpeeds speed, boolean fieldRelative, boolean openLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speed, getHeading()) : speed);
    setModuleStates(swerveModuleStates, openLoop);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false, false);
  }

  /**
   * Set the module states. States will be desaturated before setting the state.
   *
   * @param desiredStates the desired states of the swerve modules
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxModuleSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    poseEstimator.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    poseEstimator.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  @Log.NT
  public Rotation2d getGyroYaw() {
    return gyro.getYaw();
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public Command teleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
    return run(
        () -> {
          /* Get Values, Deadband*/
          double translationVal = MathUtil.applyDeadband(x.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(y.getAsDouble(), Constants.stickDeadband);
          double rotationVal = MathUtil.applyDeadband(theta.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxModuleSpeed),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              true,
              true);
        });
  }

  @Override
  public void periodic() {
    poseEstimator.update(getGyroYaw(), getModulePositions());
    if (Constants.apriltagsEnabled) {
      for (var camera : cameras) {
        var result = camera.update();
        if (result.isPresent()) {
          poseEstimator.addVisionMeasurement(
              result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
        }
      }
    }
    var pose = getPose();
    field.setRobotPose(pose);
    SmartDashboard.putNumberArray(
        "Pose", new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Distance", mod.getPosition().distanceMeters);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
