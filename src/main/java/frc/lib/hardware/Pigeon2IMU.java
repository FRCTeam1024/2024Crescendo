package frc.lib.hardware;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Objects;

public class Pigeon2IMU implements IMU, Sendable {
  private Pigeon2 m_pigeon;

  public Pigeon2IMU(Pigeon2 pigeon) {
    m_pigeon = Objects.requireNonNull(pigeon, "Parameter 'pigeon' cannot be null!");
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw().getValueAsDouble());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(m_pigeon.getPitch().getValueAsDouble());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll().getValueAsDouble());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    m_pigeon.setYaw(yaw.getDegrees());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_pigeon.initSendable(builder);
  }
}
