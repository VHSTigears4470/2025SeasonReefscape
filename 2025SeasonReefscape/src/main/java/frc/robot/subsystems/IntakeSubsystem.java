// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  //Intake - Back, Shoot - Front
  private final SparkMax m_intakeMotorLeft; 
  private final SparkMax m_intakeMotorRight; 
  private final SparkMax m_shootMotorLeft; 
  private final SparkMax m_shootMotorRight; 

  private final RelativeEncoder m_intakeEncoderLeft;
  private final RelativeEncoder m_intakeEncoderRight;
  private final RelativeEncoder m_shootEncoderLeft;
  private final RelativeEncoder m_shootEncoderRight;

  private final AnalogInput m_sensor; //TODO: Change datatype as applicable
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotorLeft = new SparkMax(Constants.IntakeConstants.k_intakePortLeft, MotorType.kBrushless);
    m_intakeMotorRight = new SparkMax(Constants.IntakeConstants.k_intakePortRight, MotorType.kBrushless);
    m_shootMotorLeft = new SparkMax(Constants.IntakeConstants.k_shootPortLeft, MotorType.kBrushless);
    m_shootMotorRight = new SparkMax(Constants.IntakeConstants.k_shootPortRight, MotorType.kBrushless);

    m_intakeEncoderLeft = m_intakeMotorLeft.getEncoder();
    m_intakeEncoderRight = m_intakeMotorRight.getEncoder();
    m_shootEncoderLeft= m_shootMotorLeft.getEncoder();
    m_shootEncoderRight = m_shootMotorRight.getEncoder();

    m_sensor = new AnalogInput(0); //TODO: Add constant for port if applicable
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
  return m_sensor;
 }

 public void setSmartDashboard(){
  SmartDashboard.putNumber("ShootEncoderL", m_shootEncoderLeft.getPosition() );
  SmartDashboard.putNumber("ShootEncoderR", m_shootEncoderRight.getPosition() );
  SmartDashboard.putNumber("IntakeEncoderL", m_intakeEncoderLeft.getPosition() );
  SmartDashboard.putNumber("IntakeEncoderL", m_intakeEncoderRight.getPosition() );
 }

 public void periodic(){
  setSmartDashboard();
 }
}