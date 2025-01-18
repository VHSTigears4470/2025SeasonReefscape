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
import frc.robot.Constants.VoltageConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class IntakeSubsystem extends SubsystemBase {
  /** Variables for intake motors */
  private final SparkMax m_botMotor;
  private final SparkMax m_topMotor;
  private final SparkMax m_armMotor;
  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_bottomEncoder;
  private final RelativeEncoder m_armEncoder;
  

  public IntakeSubsystem() {


    m_botMotor = new SparkMax(0, MotorType.kBrushless);
    m_topMotor = new SparkMax(0, MotorType.kBrushless);
    m_armMotor = new SparkMax(0, MotorType.kBrushless);
    m_topEncoder = m_topMotor.getEncoder();
    m_bottomEncoder = m_botMotor.getEncoder();
    m_armEncoder = m_armMotor.getEncoder();
  }

  
  public double getTopEncoder() {
    return (m_topEncoder.getPosition() * Math.PI) / 21;
  }
  
  public double getBottomEncoder() {
    return (m_bottomEncoder.getPosition() * Math.PI) / 21;
  }

  public double getArmState() {
    return (m_armEncoder.getPosition() * Math.PI) / 21;
  }

  public void setArmState(ARM_STATE, desiredState) {
    if (desiredState == ArmState.forward) {
      currState = desiredState;
      desiredReferencePosition = highestPos;
    } else if (desiredState == ArmState.backward) {
      currState = desiredState;
      desiredReferencePosition = lowestPos;
    }
    pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }

public void setHeightState(ELEVATOR_STATE desiredState) {
    if (desiredState == ArmState.FORWARD) {
      currState = desiredState;
      desiredReferencePosition = FORWARD;
    } else if (desiredState == ELEVATOR_STATE.DOW) {
      currState = desiredState;
      desiredReferencePosition = lowestPos;
    } else if (desiredState == ELEVATOR_STATE.CLIMB) {
      currState = desiredState;
      desiredReferencePosition = ElevatorConstants.CLIMB_HEIGHT;
    }
    pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }
  public void setSmartDashboard(){
      SmartDashboard.putNumber("TopEncoder", m_topEncoder.getPosition());
      SmartDashboard.putNumber("BottomEncoder", m_bottomEncoder.getPosition());
      SmartDashboard.putNumber("ArmEncoder", m_armEncoder.getPosition());
  }
  
  public void resetEncoder(){
    m_topEncoder.setPosition(0);
    m_bottomEncoder.setPosition(0);
    m_armEncoder.setPosition(0);
  }

  public void setTopIntakeVoltage() {
    m_topMotor.setVoltage(-VoltageConstants.FAST_INTAKE_VOLTAGE);
  }


   public void setBottomIntakeVoltage() {
    m_botMotor.setVoltage(-VoltageConstants.SLOW_INTAKE_VOLTAGE);
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