package frc.lib.hardware;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;

public class Pigeon1IMU implements IMU {
  private PigeonIMU m_pigeon;

  public Pigeon1IMU(PigeonIMU pigeon) {
    m_pigeon = Objects.requireNonNull(pigeon, "Parameter 'pigeon' cannot be null!");
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(m_pigeon.getPitch());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll());
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    m_pigeon.setYaw(-yaw.getDegrees());
  }
}
