package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TwoJointArmKinematics {
  private final double lengthOne;
  private final double lengthTwo;

  public TwoJointArmKinematics(double lengthOne, double lengthTwo) {
    this.lengthOne = lengthOne;
    this.lengthTwo = lengthTwo;
  }

  public Translation2d getJointOneTranslation(Rotation2d jointAngle) {
    return new Translation2d(lengthOne, jointAngle);
  }

  public Translation2d getJointTwoTranslation(Rotation2d jointAngle) {
    return new Translation2d(lengthTwo, jointAngle);
  }

  public Translation2d getEndEffectorTranslation(
      Rotation2d jointOneAngle, Rotation2d jointTwoAngle) {
    return getJointOneTranslation(jointOneAngle).plus(getJointTwoTranslation(jointTwoAngle));
  }
}
