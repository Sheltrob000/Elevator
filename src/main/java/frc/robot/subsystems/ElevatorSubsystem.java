// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {



  private static final double gearing = 4 * 4 * 4 / 30;
  private static final double radius = Units.inchesToMeters(2);

  private final CANSparkMax motor;
  private final TrapezoidProfile profile;
  private final ElevatorFeedforward feedforward;
  private final PIDController pidController;




  public ElevatorSubsystem(int deviceID) {
    motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    motor.getEncoder().setPositionConversionFactor(2 * Math.PI * radius/ gearing);
    motor.getEncoder().setVelocityConversionFactor(Math.PI * radius / gearing / 60.0);


  }



  public double getPositionMeters() {
    return motor.getEncoder().getPosition();
  }

  public double getVelovityMetersPerSec() {
    return motor.getEncoder().getVelocity();
  }

  public void setPositionMeters() {

  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
