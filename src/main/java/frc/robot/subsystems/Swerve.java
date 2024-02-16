package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import monologue.Logged;
import monologue.Annotations.Log;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Swerve extends SubsystemBase implements Logged {
  private ProfiledPIDController headingController;
  private Rotation2d storedGoalHeading;
  private Pose2d speakerPose;
  private SwerveDrivePoseEstimator poseEstimator;
  private List<PhotonPoseEstimator> cameras;
  private SwerveModule[] mSwerveMods;
  private PigeonIMU gyro;
  private Field2d field = new Field2d();

  public Swerve() {
    SmartDashboard.putData(field);
    gyro = new PigeonIMU(Constants.Swerve.pigeonID);
    gyro.configAllSettings(new PigeonIMUConfiguration());
    gyro.setYaw(0);

    headingController = new ProfiledPIDController(
                          Constants.Swerve.headingkP,
                          Constants.Swerve.headingkI,
                          Constants.Swerve.headingkD,
                          new TrapezoidProfile.Constraints(
                            Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularAcceleration));
    
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    storedGoalHeading = new Rotation2d();
    

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

  @Log.NT
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
    
    storedGoalHeading = new Rotation2d();
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public Rotation2d getHeadingToTarget() {
    int targetID;
    Pose2d targetPose;
    Rotation2d targetHeading; 
    var alliance = DriverStation.getAlliance();
    if(alliance.isEmpty()) {
      return storedGoalHeading;
    } 
    else if(alliance.get().equals(Alliance.Red)) {
      targetID = 4;
    }
    else {
      targetID = 7;
    }
    targetPose = Constants.kOfficialField.getTagPose(targetID).get().toPose2d();
    return targetPose.minus(getPose()).getTranslation().getAngle();
  }

  public Command teleopDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
    return run(
        () -> {
          /* Reset goal heading */
          storedGoalHeading = getHeading();
          
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

  public Command teleopHeadingDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier hx, DoubleSupplier hy, BooleanSupplier aim) {
    return run(
        () -> {
          /* Get Values, Deadband Translation*/
          double translationVal = MathUtil.applyDeadband(x.getAsDouble(), Constants.stickDeadband);
          double strafeVal = MathUtil.applyDeadband(y.getAsDouble(), Constants.stickDeadband);
          double headingX = hx.getAsDouble();
          double headingY = hy.getAsDouble();

          /* Create a Rotation2D to represent desired heading */
          Rotation2d goalHeading = new Rotation2d(headingX, headingY);

          /* Polar Deadband Heading without scaling */
          double r = Math.sqrt(headingX*headingX + headingY*headingY);
          if(r<0.8) {
            goalHeading = storedGoalHeading;
          }
          else {
            storedGoalHeading = goalHeading;
          }

          /* Override joystick and aim at goal if requested */
          if(aim.getAsBoolean()){
            goalHeading = getHeadingToTarget();
            storedGoalHeading = goalHeading;
          }

          /*  Calculate rotation velocity using heading controller */
          double rotationVelocity = headingController.calculate(MathUtil.angleModulus(getHeading().getRadians()), 
                                                                MathUtil.angleModulus(goalHeading.getRadians()));
          
          /* Drive */
          drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxModuleSpeed),
              rotationVelocity,true,true);
        }
    );
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
    log("heading setpoint", headingController.getSetpoint().position);
    log("heading", getHeading().getRadians());
  }
}
