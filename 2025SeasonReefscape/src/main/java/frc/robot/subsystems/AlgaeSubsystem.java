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
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;

public class AlgaeSubsystem extends SubsystemBase {
  /** Algae Subsystem */
   private final SparkMax m_algaeTopMotor;
   private final SparkMax m_algaeArmMotor; 
   private final RelativeEncoder m_algaeTopEncoder;
   private final RelativeEncoder m_algaeArmEncoder;
  // set positions equal to something later
   private double d_desiredReferencePosition;
   private ALGAE_ARM_STATE e_armState;

  public AlgaeSubsystem(){
    m_algaeTopMotor = new SparkMax(Constants.AlgaeConstants.k_algaeTopID, MotorType.kBrushless);
    m_algaeArmMotor = new SparkMax(Constants.AlgaeConstants.k_algaeArmID, MotorType.kBrushless);

    m_algaeTopEncoder = m_algaeTopMotor.getEncoder();
    m_algaeArmEncoder = m_algaeArmMotor.getEncoder();
    resetEncoders();

    setAlgaeState(ALGAE_ARM_STATE.RAISED);
  }
  
  //All get___Encoder methods return value in radians
  public double getAlgaeTopEncoder(){
    return (m_algaeTopEncoder.getPosition() * Math.PI) / 21;
  }
  
  public double getAlgaeArmEncoder(){
    return (m_algaeArmEncoder.getPosition() * Math.PI) / 21;
  }

  public ALGAE_ARM_STATE getArmState(){
    return e_armState;
  }

  public double getDesiredPos(){
    return d_desiredReferencePosition;
  }

  public void setAlgaeState(ALGAE_ARM_STATE p_desiredState) {
    if (p_desiredState == ALGAE_ARM_STATE.RAISED) {
      e_armState = p_desiredState;
      d_desiredReferencePosition = AlgaeConstants.k_raisedArmPos;
    } else if (p_desiredState == ALGAE_ARM_STATE.CENTERED) {
      e_armState = p_desiredState;
      d_desiredReferencePosition = AlgaeConstants.k_centeredArmPos;
    } else if (p_desiredState == ALGAE_ARM_STATE.LOWERED) {
      e_armState = p_desiredState;
      d_desiredReferencePosition = AlgaeConstants.k_loweredArmPos;
    }
    //pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
    //Apply above in config
  }
  
  public void intake() {
    m_algaeTopMotor.setVoltage(AlgaeConstants.k_intakeVoltage);
  }

  public void dispense() {
    m_algaeTopMotor.setVoltage(AlgaeConstants.k_dispenseVoltage);
  }

  public void resetEncoders(){
    m_algaeTopEncoder.setPosition(0);
    m_algaeArmEncoder.setPosition(0);
  }

  public void setSmartDashboard(){
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("AlgaeTopEncoder", getAlgaeTopEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("AlgaeArmEncoder", getAlgaeArmEncoder() * 180 / Math.PI);
  }
  
  public void stop(){
    m_algaeTopMotor.stopMotor();
    m_algaeArmMotor.stopMotor();
  }

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