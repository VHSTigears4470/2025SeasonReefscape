// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotorLeft; //Back Motor
  private final SparkMax m_intakeMotorRight; //Back Motor
  private final SparkMax m_shootMotorLeft; // Front Motor
  private final SparkMax m_shootMotorRight; // Front Motor

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

  //TODO SET MOTOR DEFAULT 

    //TODO;CHECK IF INTAKE IS ON
    m_intakeEncoderLeft = m_intakeMotorLeft.getEncoder();
    m_intakeEncoderRight = m_intakeMotorRight.getEncoder();
    m_shootEncoderLeft= m_shootMotorLeft.getEncoder();
    m_shootEncoderRight = m_shootMotorRight.getEncoder();

    sensor = new AnalogInput(0);
  }

 public double getIntakeEncoderLeft(){
  return Math.PI * m_intakeEncoderLeft.getPosition() / 21;
 }

public double getIntakeEncoderRight(){
  return Math.PI * m_intakeEncoderRight.getPosition() / 21;
}

public double getShootEncoderRight(){
  return Math.PI * m_shootEncoderRight.getPosition() / 21;
}

public double getShootEncoderLeft(){
  return Math.PI * m_shootEncoderLeft.getPosition() / 21;
}

 public AnalogInput SensorValue(){
  // COME BACK TO THIS LATER: WE DONT KNOW WHAT SENSOR WE ARE USING YET
  return sensor;
 }

 public void SmartDashboard() {
  SmartDashboard.putNumber("ShootEncoderL", m_shootEncoderLeft.getPosition() );
  SmartDashboard.putNumber("ShootEncoderR", m_shootEncoderRight.getPosition() );
  SmartDashboard.putNumber("IntakeEncoderL", m_intakeEncoderLeft.getPosition() );
  SmartDashboard.putNumber("IntakeEncoderL", m_intakeEncoderRight.getPosition() );
 }
}