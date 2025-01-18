// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
  /** Algae Subsystem */
   private final SparkMax m_algaeTopMotorSparkMax;
   private final SparkMax m_algaeArmMotorSparkMax; 
   private final RelativeEncoder m_algaeTopEncoder;
   private final RelativeEncoder m_algaeArmEncoder;
   // set positions equal to something later
   private double highestPos; 
   private double lowestPos; 
   private double desiredReferencePosition;
   private ARM_STATE e_armState;

  public AlgaeSubsystem(){
    m_algaeMotorSparkMax = new SparkMax(Constants.AlgaeConstants.k_climbMotorID, MotorType.kBrushless);
    m_algaeEncoder = m_algaeTopMotorSparkMax.getEncoder();
  }

  public double getAlgaeTopEncoder(){
    return (m_algaeTopEncoder.getPosition() * Math.PI) / 21;
  }
  
  public double getArmAlgaeEncoder(){
    return (m_algaeArmEncoder.getPosition() * Math.PI) / 21;
  }

  public ARM_STATE getArmState() {
    return e_armState;
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
    m_algaeTopMotor.setVoltage(-AlgaeConstants.k_fastVoltage);
  }

  public void shootSlow() {
    m_algaeTopMotor.setVoltage(AlgaeConstants.k_slowVoltage);
  }

  public void shootFast() {
    m_algaeTopMotor.setVoltage(AlgaeConstants.k_fastVoltage);
  }
  
  public void resetAlgaeEncoders(){
    m_algaeTopEncoder.setPosition(0);
    m_algaeArmEncoder.setPosition(0);
  }

  public void setSmartDashboard(){
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("AlgaeTopEncoder", getAlgaeTopEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("AlgaeArmEncoder", getAlgaeArmEncoder() * 180 / Math.PI);
  }

  public void resetAlgaeEncoder(){
    m_algaeTopEncoder.setPosition(0);
    m_algaeArmEncoder.setPosition(0);
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