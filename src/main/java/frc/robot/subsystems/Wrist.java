package frc.robot.subsystems;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final CANSparkMax wristMotor = new CANSparkMax(kWristId, MotorType.kBrushed);
  private final AbsoluteEncoder wristEncoder;


  public Wrist() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristEncoder.setZeroOffset(0.5 + 0.117);
    wristMotor.burnFlash();
  }

  public void setOutput(double output) {
    wristMotor.set(output);
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  public void stop() {
    setOutput(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist position", getPosition());
  }
}
