// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotorLeft;
  private final SparkMax m_intakeMotorRight;
  private final SparkMax m_shootMotorLeft;
  private final SparkMax m_shootMotorRight;

  private final RelativeEncoder m_intakeEncoderLeft;
  private final RelativeEncoder m_intakeEncoderRight;
  private final RelativeEncoder m_shootEncoderLeft;
  private final RelativeEncoder m_shootEncoderRight;

  private final AnalogInput sensor; //TODO: Change datatype
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //TODO: Add channels in constants
    m_intakeMotorLeft = new SparkMax(0, MotorType.kBrushless);
    m_intakeMotorRight = new SparkMax(0, MotorType.kBrushless);
    m_shootMotorLeft = new SparkMax(0, MotorType.kBrushless);
    m_shootMotorRight = new SparkMax(0, MotorType.kBrushless);

    m_intakeEncoderLeft = m_intakeMotorLeft.getEncoder();
    m_intakeEncoderRight = m_intakeMotorRight.getEncoder();
    m_shootEncoderLeft= m_shootMotorLeft.getEncoder();
    m_shootEncoderRight = m_shootMotorRight.getEncoder();

    sensor = new AnalogInput(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
