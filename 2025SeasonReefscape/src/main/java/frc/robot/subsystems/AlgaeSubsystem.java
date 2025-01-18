// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.ARM_STATE;

/** Algae Subsystem */
public class AlgaeSubsystem extends SubsystemBase {
  /** Variables for intake motors */
   private final SparkMax m_algaeTopMotorSparkMax;
   private final SparkMax m_algaeArmMotorSparkMax; 
   private final RelativeEncoder m_algaeTopEncoder;
   private final RelativeEncoder m_algaeArmEncoder;
   
   // set positions equal to something later
   private double highestPos; 
   private double lowestPos; 
   private double centeredPos;
   private double desiredReferencePosition;
   private ARM_STATE e_armState;

   // Constructor:
  public AlgaeSubsystem() {
    m_algaeTopMotorSparkMax = new SparkMax(Constants.AlgaeConstants.k_topID, MotorType.kBrushless);
    m_algaeTopEncoder = m_algaeTopMotorSparkMax.getEncoder();
    m_algaeArmMotorSparkMax = new SparkMax(Constants.AlgaeConstants.k_topID, MotorType.kBrushless);
    m_algaeArmEncoder = m_algaeTopMotorSparkMax.getEncoder();
  }

  //All get___AlgaeEncoder methods return value in radians
  public double getAlgaeTopEncoder() {
    return (m_algaeTopEncoder.getPosition() * Math.PI) / 21;
  }
  
  public double getAlgaeArmEncoder() {
    return (m_algaeArmEncoder.getPosition() * Math.PI) / 21;
  }

  public ARM_STATE getArmState() {
    return e_armState;
  }

  public void resetAlgaeEncoders() {
    m_algaeTopEncoder.setPosition(0);
    m_algaeArmEncoder.setPosition(0);
  }

  public void setArmState(ARM_STATE desiredState) {
    double desiredReferencePosition;
    if (desiredState == ARM_STATE.RAISED) {
      e_armState = desiredState;
      desiredReferencePosition = highestPos;
    } else if (desiredState == ARM_STATE.CENTERED) {
      e_armState = desiredState;
      desiredReferencePosition = centeredPos;
    } else if (desiredState == ARM_STATE.LOWERED) {
        e_armState = desiredState;
      desiredReferencePosition = lowestPos;
    }
    //pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }


  public void intake() {
    m_algaeTopMotorSparkMax.setVoltage(-AlgaeConstants.k_fastVoltage);
  }

  public void shootSlow() {
    m_algaeTopMotorSparkMax.setVoltage(AlgaeConstants.k_slowVoltage);
  }

  public void shootFast() {
    m_algaeTopMotorSparkMax.setVoltage(AlgaeConstants.k_fastVoltage);
  }
  
// Dashboard methods

  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("AlgaeTopEncoder", getAlgaeTopEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("AlgaeArmEncoder", getAlgaeArmEncoder() * 180 / Math.PI);
  }


  //Still need to apply other methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    
    // This method will be called once per scheduler run during simulation
  }
}