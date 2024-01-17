package frc.lib.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMU {
  Rotation2d getYaw();

  Rotation2d getPitch();

  Rotation2d getRoll();

  void setYaw(Rotation2d yaw);
}
