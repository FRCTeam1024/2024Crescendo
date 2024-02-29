package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class SwerveModule implements Logged {
  public final int moduleNumber;
  @Log.Once private final Rotation2d angleOffset;

  private final TalonFX mAngleMotor;
  private final TalonFX mDriveMotor;
  private final CANcoder angleEncoder;

  private final StatusSignal<Double> angleMotorPosition;
  private final StatusSignal<Double> angleMotorVelocity;
  private final StatusSignal<Double> angleMotorTemperature;

  private final StatusSignal<Double> driveMotorPosition;
  private final StatusSignal<Double> driveMotorVelocity;
  private final StatusSignal<Double> driveMotorVoltage;
  private final StatusSignal<Double> driveMotorTemperature;

  private final StatusSignal<Double> angleEncoderAbsolutePosition;

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
  private final VoltageOut driveVoltage = new VoltageOut(0);

  /* angle motor control requests */
  private final PositionVoltage anglePositionRequest = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    angleEncoderAbsolutePosition = angleEncoder.getAbsolutePosition();

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
    mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    angleMotorPosition = mAngleMotor.getPosition();
    angleMotorVelocity = mAngleMotor.getVelocity();
    angleMotorTemperature = mAngleMotor.getDeviceTemp();
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
    mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.getConfigurator().setPosition(0.0);
    driveMotorPosition = mDriveMotor.getPosition();
    driveMotorVelocity = mDriveMotor.getVelocity();
    driveMotorVoltage = mDriveMotor.getMotorVoltage();
    driveMotorTemperature = mDriveMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        angleMotorPosition,
        angleMotorVelocity,
        driveMotorPosition,
        driveMotorVelocity,
        driveMotorVoltage,
        angleEncoderAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(4, angleMotorTemperature, driveMotorTemperature);
    if (Constants.disableUnusedSignals) {
      mAngleMotor.optimizeBusUtilization();
      mDriveMotor.optimizeBusUtilization();
      angleEncoder.optimizeBusUtilization();
    }
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    var angleError = desiredState.angle.minus(getAngle());
    // Clamp scale factor to (0, 1) to prevent reversing
    // This shouldn't ever happen but just in case
    var velocityInDesiredDirection =
        MathUtil.clamp(angleError.getCos(), 0, 1) * desiredState.speedMetersPerSecond;

    setAngle(desiredState.angle);
    setSpeed(velocityInDesiredDirection, isOpenLoop);
  }

  private void setAngle(Rotation2d angle) {
    mAngleMotor.setControl(anglePositionRequest.withPosition(angle.getRotations()));
  }

  /**
   * Sets the desired speed of the drive wheel.
   *
   * @param speedMetersPerSecond Desired speed of the drive wheel in meters per second.
   * @param isOpenLoop Whether to use open loop or closed loop control. If this is true, the duty
   *     cycle output will be ${@code speedMetersPerSecond / maxSpeed}.
   */
  private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
    // Convert linear speed of wheel to motor speed
    var requestedVelocityRPS = wheelMeterToMotorRot(speedMetersPerSecond);
    // Calculate motor velocity required to hold wheel still at the current azimuth velocity
    // Don't compensate if requested velocity is 0 - just stop the motor
    double compensationVelocity = 0;
    if (speedMetersPerSecond != 0) {
      compensationVelocity =
          mAngleMotor.getVelocity().getValueAsDouble() * Constants.Swerve.azimuthCouplingRatio;
    }
    var outputVelocity = requestedVelocityRPS + compensationVelocity;

    if (isOpenLoop) {
      driveDutyCycleRequest.Output =
          outputVelocity / wheelMeterToMotorRot(Constants.Swerve.maxModuleSpeed);
      mDriveMotor.setControl(driveDutyCycleRequest);
    } else {
      driveVelocityRequest.Velocity = outputVelocity;
      // The feedforward still uses meters per second
      driveVelocityRequest.FeedForward = driveFeedForward.calculate(speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocityRequest);
    }
  }

  /**
   * Commands the swerve module to an angle and drive voltage. Does <i>not</i> optimize the angle.
   * This is used for system identification testing.
   *
   * @param angle the azimuth angle.
   * @param voltage the voltage for the drive motor.
   */
  public void setAngleAndVoltage(Rotation2d angle, double voltage) {
    setAngle(angle);
    mDriveMotor.setControl(driveVoltage.withOutput(voltage));
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        motorRotToWheelMeter(mDriveMotor.getVelocity().getValue()), getAngle());
  }

  public SwerveModulePosition getPosition() {
    var driveRotations = mDriveMotor.getPosition().getValueAsDouble();
    // Calculate how many drive rotations were caused by azimuth coupling
    var azimuthCompensationDistance =
        getAngle().getRotations() * Constants.Swerve.azimuthCouplingRatio;
    // Subtract the "false" rotations from the recorded rotations to get the rotations that caused
    // wheel motion
    var trueDriveRotations = driveRotations - azimuthCompensationDistance;

    return new SwerveModulePosition(motorRotToWheelMeter(trueDriveRotations), getAngle());
  }

  @Log.NT
  public double getDriveVoltage() {
    return driveMotorVoltage.refresh().getValue();
  }

  public void updateLog() {
    log("Azimuth Motor Velocity", angleMotorVelocity.refresh().getValue());
    log("Azimuth Motor Temperature", angleMotorTemperature.refresh().getValue());
    log("Drive Motor Temperature", driveMotorTemperature.refresh().getValue());
  }

  public static double wheelMeterToMotorRot(double wheelMeters) {
    return Conversions.metersToRotations(wheelMeters, Constants.Swerve.wheelCircumference)
        * Constants.Swerve.driveGearRatio;
  }

  public static double motorRotToWheelMeter(double motorRot) {
    return Conversions.RPSToMPS(
        motorRot / Constants.Swerve.driveGearRatio, Constants.Swerve.wheelCircumference);
  }
}
