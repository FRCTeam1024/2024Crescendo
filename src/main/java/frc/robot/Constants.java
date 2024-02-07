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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final double stickDeadband = 0.02;

  public static final boolean apriltagsEnabled = false;

  public static final AprilTagFieldLayout kOfficialField =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // For now, comp bot is the 2023 robot, practice bot is the practice bot
  public static final String compBotSerialNum = "03264208";
  public static final String practiceBotSerialNum = "03241508";
  public static final boolean isPracticeBot =
      RobotController.getSerialNumber().equals(practiceBotSerialNum);

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
    public static final double wheelCircumference = Units.inchesToMeters(Math.PI * 4 * 0.931755098);

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

    public static final int driveCurrentLimit = 35;
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
    public static final double maxModuleSpeed = 4.5; // TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

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
          Rotation2d.fromDegrees(isPracticeBot ? -126.562500 : 0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? -168.046875 : 0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? -21.005859 : 0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(isPracticeBot ? 11.777344 : 0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class IntakeConstants {
    public static final int intakeMotorId = 43;
    public static final boolean isInverted = false;

    // Feedforward
    public static final double kS = 0;
    public static final double kV = .123;
    public static final double kA = 0;

    // Feedback
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class ClimberConstants {
    public static final int ClimberMotorAId = 46;
    public static final int ClimberMotorBId = 47;
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
  }

  public static final
  class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
    // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

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
