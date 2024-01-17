package frc.lib.hardware;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Objects;

public class Pigeon1IMU implements IMU, Sendable {
  private WPI_PigeonIMU m_pigeon;

  public Pigeon1IMU(WPI_PigeonIMU pigeon) {
    m_pigeon = Objects.requireNonNull(pigeon, "Parameter 'pigeon' cannot be null!");
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-m_pigeon.getYaw());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(-m_pigeon.getPitch());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(-m_pigeon.getRoll());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    m_pigeon.setYaw(-yaw.getDegrees());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_pigeon.initSendable(builder);
  }
}
