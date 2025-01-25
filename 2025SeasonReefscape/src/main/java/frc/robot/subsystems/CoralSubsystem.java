// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ARM_STATE;

public class CoralSubsystem extends SubsystemBase {
  /** Variables for intake motors */
  private final SparkMax m_botMotor;
  private final SparkMax m_topMotor;
  private final SparkMax m_armMotor;
  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_botEncoder;
  private final RelativeEncoder m_armEncoder;
  private ARM_STATE e_armState;
  //add sensor if necessary

  public CoralSubsystem() {
    e_armState = ARM_STATE.FORWARD;
    m_botMotor = new SparkMax(IntakeConstants.k_botID, MotorType.kBrushless);
    m_topMotor = new SparkMax(IntakeConstants.k_topID, MotorType.kBrushless);
    m_armMotor = new SparkMax(IntakeConstants.k_armID, MotorType.kBrushless);
    m_topEncoder = m_topMotor.getEncoder();
    m_botEncoder = m_botMotor.getEncoder();
    m_armEncoder = m_armMotor.getEncoder();
    resetEncoders();
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
  public ARM_STATE getArmState() {
    return e_armState;
  }


  public void intake() {
    m_topMotor.setVoltage(-IntakeConstants.k_fastVoltage);
    m_botMotor.setVoltage(-IntakeConstants.k_fastVoltage);
  }

//Shoots slow by decreasing the voltage of the top and bottom motors
  public void shootSlow() {
    m_topMotor.setVoltage(IntakeConstants.k_slowVoltage);
    m_botMotor.setVoltage(IntakeConstants.k_slowVoltage);
  }

//Shoots fast by increasing the voltage of the top and bottom motors
  public void shootFast() {
    m_topMotor.setVoltage(IntakeConstants.k_fastVoltage);
    m_botMotor.setVoltage(IntakeConstants.k_fastVoltage);
  }

  //TODO: Finish
  public void setArmState(ARM_STATE desiredState) {
    if (desiredState == ARM_STATE.FORWARD) {
      e_armState = desiredState;
      //desiredReferencePosition = highestPos;
    } else if (desiredState == ARM_STATE.BACKWARD) {
      e_armState = desiredState;
      //desiredReferencePosition = lowestPos;
    }
    //pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }

//sets the encoders to default values
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("TopEncoder", getTopEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("BottomEncoder", getBotEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("ArmEncoder", getArmEncoder() * 180 / Math.PI);
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