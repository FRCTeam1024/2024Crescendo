package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import org.photonvision.PhotonCamera;

public final class Constants {
  public static final double stickDeadband = 0.04;

  public static final boolean apriltagsEnabled = true;

  public static final boolean disableUnusedSignals = true;

  public static final boolean enableDriverCam = false;

  public static final AprilTagFieldLayout kOfficialField =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // For now, comp bot is the 2023 robot, practice bot is the practice bot
  public static final String compBotSerialNum = "03264208";
  public static final String practiceBotSerialNum = "03241508";
  public static final boolean isPracticeBot =
      RobotController.getSerialNumber().equals(practiceBotSerialNum) || RobotBase.isSimulation();

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
            COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /** Drive motor rotations per rotation of azimuth */
    public static final double azimuthCouplingRatio = 50.0 / 14.0;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.75);
    public static final double wheelBase = Units.inchesToMeters(21.75);
    // 4 inches * correction factor obtained via measured distance
    public static final double wheelCircumference = Units.inchesToMeters(isPracticeBot ? Math.PI * 4 : Math.PI * 4);

    /* Heading Control PID Values */
    public static final double headingkP = 2;
    public static final double headingkI = 0;
    public static final double headingkD = 0;

    /* Heading Feedforward Values */
    public static final double headingkS = 0;
    public static final double headingkV = 1;
    public static final double headingkA = 0;

    /* Heading Goal Range */
    public static final double headingGoalRange = Units.degreesToRadians(1);

    /* Translate Control PID Values */
    public static final double translatekP = 4;
    public static final double translatekI = 0;
    public static final double translatekD = 0;

    /* Translate Feedforward Values */
    public static final double translatekS = 0;
    public static final double translatekV = 1;
    public static final double translatekA = 0;

    /* Heading Goal Range */
    public static final double translateGoalRange = 0.025;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 40;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.2;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    // kV was obtained from a linear regression on empirical voltage/velocity data.
    public static final double driveKS = 0.32;
    public static final double driveKV = 2.57847;
    public static final double driveKA = 0.0;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxModuleSpeed = 4.5;

    /** Meters per Second per Second */
    public static final double maxAcceleration = 6;

    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /** Radians per Second per Second */
    public static final double maxAngularAcceleration = 15;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? -126.562500 : 56.162109);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? -168.046875 : 130.693359);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? -21.005859 : -121.201172);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? 11.777344 : -160.400391);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class CameraConstants {
    public static final String frontCameraName =
        isPracticeBot ? "Arducam_OV9281_5" : "Arducam_OV9281_1";
    public static final PhotonCamera frontCamera = new PhotonCamera(frontCameraName);
    public static final Transform3d frontCamTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(isPracticeBot ? -11.25 : -11.25),
                Units.inchesToMeters(isPracticeBot ? 6.5 : 6.5),
                Units.inchesToMeters(isPracticeBot ? 19.25 : 19.25)),
            new Rotation3d(
                Units.degreesToRadians(isPracticeBot ? 180 : 180),
                Units.degreesToRadians(isPracticeBot ? -25 : -25),
                Units.degreesToRadians(isPracticeBot ? 0 : 0)));

    public static final String rearCameraName =
        isPracticeBot ? "Arducam_OV9281_4" : "Arducam_OV9281_3";
    public static final PhotonCamera rearCamera = new PhotonCamera(rearCameraName);
    public static final Transform3d rearCamTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(isPracticeBot ? -13.25 : -13.25),
                Units.inchesToMeters(isPracticeBot ? -6.5 : -6.5),
                Units.inchesToMeters(isPracticeBot ? 20.875 : 20.875)),
            new Rotation3d(
                Units.degreesToRadians(isPracticeBot ? 180 : 180),
                Units.degreesToRadians(isPracticeBot ? -18 : -24),
                Units.degreesToRadians(isPracticeBot ? 180 : 180)));
  }

  public static final class WristConstants {
    // ~100 degrees of range

    public static final int kWristId = 42;

    public static final int kQuadEncoderAPin = 2;
    public static final int kQuadEncoderBPin = 3;
    public static final int kAbsEncoderPin = 5;

    /**
     * Difference between what the absolute encoder reads and what we want that angle to be, in
     * radians To find this, set the offset to 0 and read the measured position of the wrist when
     * the wrist is at 0
     */
    public static final double kOffsetAtLowerHardStop = isPracticeBot ? -2.741 : 1.04;

    public static final double kHardStopToMaxExtension = -0.511786;
    public static final double kHardStopToCOG = 0;

    public static final double kCOGZeroToMaxExtension = 0;

    public static final double kMinPosition = Units.degreesToRadians(0);
    public static final double kMaxPosition = isPracticeBot ? 2.28 : 2.4429;

    // 3-3-4 maxplanetary, 72:64 gearing with wrist
    public static final double kMotorToWristRatio = 3.0 * 3.0 * 4.0 * (72 / 64);
    public static final int kQuadTicks = 2048;

    public static final double kMaxVelocityRadiansPerSecond = Units.degreesToRadians(240);
    public static final double kMaxAccelerationRadiansPerSecondSquared =
        Units.degreesToRadians(800);

    public static final double kS = 0; // 0.25;
    public static final double kV = 0.75;
    public static final double kA = 0.0;
    public static final double kG = 0.36;

    public static final double kP = isPracticeBot ? 5 : 7.5;
    public static final double kI = 0.0;
    public static final double kD = isPracticeBot ? 0 : 0.05;
  }

  public static final class ArmConstants {
    public static final int kWristId = 41;

    public static final int kQuadEncoderAPin = 6;
    public static final int kQuadEncoderBPin = 7;
    public static final int kAbsEncoderPin = 9;

    /**
     * Difference between what the absolute encoder reads and what we want that angle to be, in
     * radians. To find this, position the arm at the hard stop and use "Position No Offset" as the
     * new offset
     */
    public static final double kOffsetAtLowerHardStop = isPracticeBot ? -0.309 : -0.873;

    public static final double kHardStopPosition = -0.5;
    public static final double kPositionOffset = kOffsetAtLowerHardStop - kHardStopPosition;

    public static final double kMinPosition = kHardStopPosition;
    public static final double kMaxPosition = Units.degreesToRadians(80) - kHardStopPosition;

    public static final double kGearboxToArmRatio = 40.0 / 12.0;
    // 3-3-4 maxplanetary, 12 t to 40 t
    public static final double kMotorToArmRatio = 3.0 * 3.0 * 4.0 * kGearboxToArmRatio;
    public static final int kQuadTicks = 2048;

    public static final double kMaxVelocityRadiansPerSecond = Units.degreesToRadians(240);
    public static final double kMaxAccelerationRadiansPerSecondSquared =
        Units.degreesToRadians(480);

    // https://www.reca.lc/arm?armMass=%7B%22s%22%3A13%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A22%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A120%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-30%2C%22u%22%3A%22deg%22%7D
    public static final double kS = 0.1;
    public static final double kV = 2.22;
    public static final double kA = 0.0;
    public static final double kG = 0.6;

    public static final double kP = 8;
    public static final double kI = 0.0;
    public static final double kD = 0.05;
  }

  public static final class IntakeConstants {
    public static final int intakeMotorId = 43;
    public static final boolean isInverted = false;
    public static final int noteSensorId = 1;

    // Output for intaking notes
    public static final double intakingSetpoint = 0.7;
    // Output for feeding to shooter
    public static final double fireSetpoint = 1.0;
  }

  public static final class ClimberConstants {
    public static final int ClimberMotorAId = 46;
    public static final int ClimberMotorBId = 47;
  }

  public static final class FeedConstants {
    public static final int FeedMotorID = 48;

    // Ratio of motor rotations to wheel rotations
    // Ratio of wheel pulley teeth to motor pulley teeth
    public static final double kFeedGearRatio = 1; // TODO: Fix

    public static final InvertedValue kFeedMotorInversionSetting = InvertedValue.Clockwise_Positive;

    // Output for intaking notes
    public static final double intakingSetpoint = 0.8;
    // Output for feeding to shooter
    public static final double fireSetpoint = 0.5;
  }

  public static final class ShooterConstants {
    public static final int kShooterAId = 44;
    public static final int kShooterBId = 45;

    // Ratio of motor rotations to wheel rotations
    // Ratio of wheel pulley teeth to motor pulley teeth
    public static final double kShooterGearRatio = 1; // TODO: Fix

    public static final InvertedValue kMotorAInversionSetting =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kMotorBInversionSetting =
        InvertedValue.CounterClockwise_Positive;

    // Feedforward
    public static final double kS = 0;
    public static final double kV = .123;
    public static final double kA = 0;

    // Feedback
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;

    // shooter limit
    public static final int shooterCurrentLimit = 40;
    public static final int shooterCurrentThreshold = 60;
    public static final double shooterCurrentThresholdTime = 0.1;
    public static final boolean shooterEnableCurrentLimit = true;
  }

  public static final class AutoConstants {
    public static final double kPXController = 3;
    public static final double kPYController = 3;
    public static final double kPThetaController = 5;

    public static final HolonomicPathFollowerConfig pathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(kPXController, 0, 0),
            new PIDConstants(kPThetaController, 0, 0),
            Swerve.maxModuleSpeed,
            Math.hypot(Swerve.wheelBase, Swerve.trackWidth),
            new ReplanningConfig(false, false));
  }
}
