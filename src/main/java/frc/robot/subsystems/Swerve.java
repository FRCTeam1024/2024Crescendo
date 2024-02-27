package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.IMU;
import frc.lib.hardware.Pigeon1IMU;
import frc.lib.hardware.Pigeon2IMU;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Swerve extends SubsystemBase implements Logged {
  private ProfiledPIDController headingController;
  private SimpleMotorFeedforward headingFeedforward;
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private SimpleMotorFeedforward translateFeedforward;
  private Rotation2d storedGoalHeading;
  private SwerveDrivePoseEstimator poseEstimator;
  private List<PhotonPoseEstimator> cameras;
  private SwerveModule[] mSwerveMods;
  private IMU gyro;
  private Field2d field = new Field2d();
  private List<Pose2d> targetPoses = new LinkedList<Pose2d>();

  public Swerve() {
    SmartDashboard.putData(field);
    gyro = createGyro();
    gyro.setYaw(new Rotation2d());

    headingController =
        new ProfiledPIDController(
            Constants.Swerve.headingkP,
            Constants.Swerve.headingkI,
            Constants.Swerve.headingkD,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration));

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingFeedforward =
        new SimpleMotorFeedforward(
            Constants.Swerve.headingkS, Constants.Swerve.headingkV, Constants.Swerve.headingkA);
    storedGoalHeading = new Rotation2d();

    xController =
        new ProfiledPIDController(
            Constants.Swerve.translatekP,
            Constants.Swerve.translatekI,
            Constants.Swerve.translatekD,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.maxModuleSpeed, Constants.Swerve.maxAcceleration));

    yController =
        new ProfiledPIDController(
            Constants.Swerve.translatekP,
            Constants.Swerve.translatekI,
            Constants.Swerve.translatekD,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.maxModuleSpeed, Constants.Swerve.maxAcceleration));

    translateFeedforward =
        new SimpleMotorFeedforward(
            Constants.Swerve.translatekS,
            Constants.Swerve.translatekV,
            Constants.Swerve.translatekA);

    List<AprilTag> tags = Constants.kOfficialField.getTags();
    for (AprilTag t : tags) {
      targetPoses.add(t.pose.toPose2d());
      if (t.ID == 5 || t.ID == 6) {
        targetPoses.add(
            t.pose.toPose2d().transformBy(new Transform2d(.7, 0, new Rotation2d(Math.PI))));
      }
      /* Set offset for pickup (source) */
      else if (t.ID == 1 || t.ID == 2 || t.ID == 9 || t.ID == 10) {
        targetPoses.add(
            t.pose.toPose2d().transformBy(new Transform2d(.7, 0, new Rotation2d(Math.PI))));
      }
      /* Set offset for stage */
      else if (t.ID >= 11 && t.ID <= 16) {
        targetPoses.add(
            t.pose.toPose2d().transformBy(new Transform2d(1.2, 0, new Rotation2d(Math.PI))));
      }
      /* Ignore speaker tags for now */
      else if (t.ID == 3 || t.ID == 4 || t.ID == 7 || t.ID == 8) {
        // Don't add anything so we wont snap here
      }
    }

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
        this::setPose,
        this::getRobotRelativeChassisSpeeds,
        this::driveRobotRelative,
        Constants.AutoConstants.pathFollowerConfig,
        this::shouldFlipPath,
        this);
    if (Constants.apriltagsEnabled) {
      cameras =
          List.of(
              new PhotonPoseEstimator(
                  Constants.kOfficialField,
                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                  Constants.CameraConstants.frontCamera,
                  Constants.CameraConstants.frontCamTransform),
              new PhotonPoseEstimator(
                  Constants.kOfficialField,
                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                  Constants.CameraConstants.rearCamera,
                  Constants.CameraConstants.rearCamTransform));
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

  public void driveVoltage(Rotation2d heading, double voltage) {
    for (var mod : mSwerveMods) {
      mod.setAngleAndVoltage(heading, voltage);
    }
  }

  public void logSysID(SysIdRoutineLog sysidLog) {
    for (var mod : mSwerveMods) {
      sysidLog
          .motor("Module-" + mod.moduleNumber)
          .value("voltage", mod.getDriveVoltage(), "Volt")
          .value("position", mod.getPosition().distanceMeters, "Meter")
          .value("velocity", mod.getState().speedMetersPerSecond, "Meter per Second");
    }
  }

  /**
   * Set the module states. States will be desaturated before setting the state.
   *
   * @param desiredStates the desired states of the swerve modules
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    log("Requested Module States", desiredStates);
    log("Closed Loop", !isOpenLoop);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxModuleSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
    }
  }

  @Log.NT(key = "Module States")
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

  @Log.NT
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  @Log.NT
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    poseEstimator.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  /*
   * Resets the robots heading such that a heading of 0 represents the
   * robot intake facing the red alliance wall if the robot is facing
   * away from the driver.
   *
   */
  public void resetHeading() {
    poseEstimator.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(
            getPose().getTranslation(),
            DriverStation.getAlliance().get().equals(Alliance.Red)
                ? new Rotation2d().unaryMinus()
                : new Rotation2d()));

    storedGoalHeading = new Rotation2d();
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

  @Log.NT
  public Rotation2d getHeadingToSpeaker() {
    int targetID;
    Pose2d targetPose;
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return storedGoalHeading;
    } else if (alliance.get().equals(Alliance.Red)) {
      targetID = 4;
    } else {
      targetID = 7;
    }
    targetPose = Constants.kOfficialField.getTagPose(targetID).get().toPose2d();
    return getPose().getTranslation().minus(targetPose.getTranslation()).getAngle();
  }

  public Optional<Pose2d> getSnapPose() {
    Pose2d currentPose = getPose();
    Pose2d targetPose = currentPose.nearest(targetPoses);

    /* Ignore if the nearest target is more than a certain distance away */
    if (targetPose.getTranslation().getDistance(currentPose.getTranslation()) > 1.5) {
      return Optional.empty();
    } else {
      return Optional.of(targetPose);
    }
  }

  public Command teleopDriveCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier rCent) {
    return run(() -> {
          /* Reset goal heading */
          storedGoalHeading = getHeading();

          /* Get Values, Deadband*/
          double translationVal = MathUtil.applyDeadband(x.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(y.getAsDouble(), Constants.stickDeadband);
          double rotationVal = MathUtil.applyDeadband(theta.getAsDouble(), Constants.stickDeadband);
          Boolean robotCentric = rCent.getAsBoolean();

          /* Invert translation controls if on Red side of field */
          var alliance = DriverStation.getAlliance();
          if (alliance.get().equals(Alliance.Red) && !robotCentric) {
            translationVal *= -1;
            strafeVal *= -1;
          }

          /* Drive */
          drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxModuleSpeed),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              !robotCentric,
              true);
        })
        .withName("teleopDriveCommand");
  }

  public Command teleopHeadingDriveCommand(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier hx,
      DoubleSupplier hy,
      BooleanSupplier aim,
      BooleanSupplier snap) {
    return run(() -> {
          /* Get Values, Deadband Translation*/
          double translationVal = MathUtil.applyDeadband(x.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(y.getAsDouble(), Constants.stickDeadband);
          double headingX = hx.getAsDouble();
          double headingY = hy.getAsDouble();
          Debouncer headingDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
          Debouncer xDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
          Debouncer yDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
          /* check for a snap pose */
          var snapPose = getSnapPose();

          /* Invert translation controls if on Red side of field */
          var alliance = DriverStation.getAlliance();
          if (alliance.get().equals(Alliance.Red)) {
            translationVal *= -1;
            strafeVal *= -1;
            headingX *= -1;
            headingY *= -1;
          }

          /* Convert Translation joystick values to module speeds */
          translationVal *= Constants.Swerve.maxModuleSpeed;
          strafeVal *= Constants.Swerve.maxModuleSpeed;

          /* Create a Rotation2D to represent desired heading */
          Rotation2d goalHeading = new Rotation2d(headingX, headingY);

          /* Polar Deadband Joystick Heading without scaling */
          double r = Math.sqrt(headingX * headingX + headingY * headingY);
          if (r < 0.8) {
            goalHeading = storedGoalHeading;
          } else {
            storedGoalHeading = goalHeading;
          }

          /* Override right joystick and aim at speaker if requested */
          if (aim.getAsBoolean()) {
            goalHeading = getHeadingToSpeaker();
            storedGoalHeading = goalHeading;
          }
          /* Override both joysticks and snap to target if requested and near a valid target */
          else if (snap.getAsBoolean() && !snapPose.isEmpty()) {
            goalHeading = snapPose.get().getRotation();
            storedGoalHeading = goalHeading;
            double goalX = snapPose.get().getX();
            double goalY = snapPose.get().getY();

            /* Calculate translation velocity using translation PID + feedforward */
            /* Stop if heading is within goal range  */
            if (Math.abs(goalX - getPose().getX()) > Constants.Swerve.translateGoalRange) {
              translationVal = xController.calculate(getPose().getX(), goalX);
              translationVal += translateFeedforward.calculate(xController.getSetpoint().velocity);
            } else {
              translationVal = 0;
            }
            if (Math.abs(goalY - getPose().getY()) > Constants.Swerve.translateGoalRange) {
              strafeVal = yController.calculate(getPose().getY(), goalY);
              strafeVal += translateFeedforward.calculate(yController.getSetpoint().velocity);
            } else {
              strafeVal = 0;
            }
          }

          /*  Calculate rotation velocity using heading controller PID + feedforward */
          /*  Stop if heading is within goal range */
          double rotationVelocity = 0;
          if (Math.abs(goalHeading.minus(getHeading()).getRadians())
              > Constants.Swerve.headingGoalRange) {
            rotationVelocity =
                headingController.calculate(
                    MathUtil.angleModulus(getHeading().getRadians()),
                    MathUtil.angleModulus(goalHeading.getRadians()));
            rotationVelocity +=
                headingFeedforward.calculate(headingController.getSetpoint().velocity);
          }

          /* Drive */
          drive(new Translation2d(translationVal, strafeVal), rotationVelocity, true, true);
        })
        .withName("teleopHeadingDriveCommand");
  }

  public boolean shouldFlipPath() {
    var result = DriverStation.getAlliance();
    if (result.isEmpty()) {
      System.out.println("Alliance was empty at auto start!");
      return false;
    }
    return result.get().equals(Alliance.Red);
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
    for (SwerveModule mod : mSwerveMods) {
      mod.updateLog();
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Distance", mod.getPosition().distanceMeters);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    log("heading setpoint", headingController.getSetpoint().position);
    log("heading", getHeading().getRadians());
    var currentCommand = getCurrentCommand();
    log("Current Command", currentCommand == null ? "None" : currentCommand.getName());
  }
}
