// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;

public class CoralSubsystem extends SubsystemBase {
  /** Variables for intake motors */
  private final SparkMax m_botMotor;
  private final SparkMax m_topMotor;
  private final SparkMax m_armMotor;
  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_botEncoder;
  private final RelativeEncoder m_armEncoder;
  private final SparkClosedLoopController m_armClosedLoopController;
  private double d_desiredReferencePosition;
  private CORAL_ARM_STATE e_armState;
  //add sensor if necessary
  public CoralSubsystem() {
    m_botMotor = new SparkMax(CoralConstants.k_botID, MotorType.kBrushless);
    m_topMotor = new SparkMax(CoralConstants.k_topID, MotorType.kBrushless);
    m_armMotor = new SparkMax(CoralConstants.k_armID, MotorType.kBrushless);
    m_topEncoder = m_topMotor.getEncoder();
    m_botEncoder = m_botMotor.getEncoder();
    m_armEncoder = m_armMotor.getEncoder();
    resetEncoders();
    m_armClosedLoopController = m_armMotor.getClosedLoopController();
    setArmState(CORAL_ARM_STATE.FORWARD);
  }

  //All get___Encoder methods return value in radians
  public double getTopEncoder() {
    return (m_topEncoder.getPosition() * Math.PI) / 21;
  }
  
  //Returns the position of the bot encoder
  public double getBotEncoder() {
    return (m_botEncoder.getPosition() * Math.PI) / 21;
  }

//Returns the position of the arm encoder
  public double getArmEncoder() {
    return (m_armEncoder.getPosition() * Math.PI) / 21;
  }

//Resets the encoder
  public void resetEncoders(){
    m_topEncoder.setPosition(0);
    m_botEncoder.setPosition(0);
    m_armEncoder.setPosition(0);
  }

//Returning the arm state
  public CORAL_ARM_STATE getArmState() {
    return e_armState;
  }


  public void intake() {
    m_topMotor.setVoltage(-CoralConstants.k_fastVoltage);
    m_botMotor.setVoltage(-CoralConstants.k_fastVoltage);
  }

//Shoots slow by decreasing the voltage of the top and bottom motors
  public void shootSlow() {
    m_topMotor.setVoltage(CoralConstants.k_slowVoltage);
    m_botMotor.setVoltage(CoralConstants.k_slowVoltage);
  }

//Shoots fast by increasing the voltage of the top and bottom motors
  public void shootFast() {
    m_topMotor.setVoltage(CoralConstants.k_fastVoltage);
    m_botMotor.setVoltage(CoralConstants.k_fastVoltage);
  }

  public void setArmState(CORAL_ARM_STATE desiredState) { //changes the position of the arm according to its desired state
    if (desiredState == CORAL_ARM_STATE.FORWARD) {
      d_desiredReferencePosition = Constants.CoralConstants.k_forwardArmPos;
    } else if (desiredState == CORAL_ARM_STATE.BACKWARD) {
      d_desiredReferencePosition = Constants.CoralConstants.k_backwardArmPos;
    }
    e_armState = desiredState;
    m_armClosedLoopController.setReference(d_desiredReferencePosition, ControlType.kPosition);
  }

   public boolean isAtDesiredPosition(){
    return (Math.abs(getArmEncoder() - d_desiredReferencePosition) < Constants.k_positionBuffer);
  } 

//sets the encoders to default values
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("TopEncoder", getTopEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("BottomEncoder", getBotEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("ArmEncoder", getArmEncoder() * 180 / Math.PI);
  }

  public void stop(){
    m_topMotor.stopMotor();
    m_botMotor.stopMotor();
    m_armMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(getArmEncoder() - d_desiredReferencePosition > Constants.k_positionBuffer)
    //   m_armMotor.set(-Constants.CoralConstants.k_coralArmSpeed);
    // else if (getArmEncoder() - d_desiredReferencePosition < -Constants.k_positionBuffer)
    //   m_armMotor.set(Constants.CoralConstants.k_coralArmSpeed);
    setSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}