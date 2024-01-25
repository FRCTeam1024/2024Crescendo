package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    // angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
    mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
    mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.getConfigurator().setPosition(0.0);
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
    mAngleMotor.setControl(anglePosition.withPosition(angle.getRotations()));
  }

  private void setSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
    // Convert linear speed of wheel to motor speed
    var requestedVelocityRPS = wheelMeterToMotorRot(speedMetersPerSecond);
    // Calculate motor velocity required to hold wheel still at the current azimuth velocity
    var compensationVelocity =
        mAngleMotor.getVelocity().getValueAsDouble() * Constants.Swerve.azimuthCouplingRatio;
    var outputVelocity = requestedVelocityRPS + compensationVelocity;

    if (isOpenLoop) {
      driveDutyCycle.Output = outputVelocity / wheelMeterToMotorRot(Constants.Swerve.maxSpeed);
      mDriveMotor.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity = outputVelocity;
      // The feedforward still uses meters per second
      driveVelocity.FeedForward = driveFeedForward.calculate(speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
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

  public static double wheelMeterToMotorRot(double wheelMeters) {
    return Conversions.metersToRotations(wheelMeters, Constants.Swerve.wheelCircumference)
        * Constants.Swerve.driveGearRatio;
  }

  public static double motorRotToWheelMeter(double motorRot) {
    return Conversions.RPSToMPS(
        motorRot / Constants.Swerve.driveGearRatio, Constants.Swerve.wheelCircumference);
  }
}
