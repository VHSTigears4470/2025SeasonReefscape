// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;

/** Algae Subsystem */
public class AlgaeSubsystem extends SubsystemBase {
  /** Algae Subsystem */
   private final SparkMax m_algaeTopMotor; /* Variables for algae motors */
   private final SparkMax m_algaeArmMotor;
   private final RelativeEncoder m_algaeTopEncoder; /** Variables for algae encoders */
   private final RelativeEncoder m_algaeArmEncoder;
   private final SparkClosedLoopController m_algaeClosedLoopController;
   
   private ALGAE_ARM_STATE e_armState;
   private double d_desiredReferencePosition;


  public AlgaeSubsystem(){
    m_algaeTopMotor = new SparkMax(Constants.AlgaeConstants.k_algaeTopID, MotorType.kBrushless); // motor controller
    m_algaeArmMotor = new SparkMax(Constants.AlgaeConstants.k_algaeArmID, MotorType.kBrushless);

    m_algaeTopEncoder = m_algaeTopMotor.getEncoder(); //recieving the encoder value
    m_algaeArmEncoder = m_algaeArmMotor.getEncoder();
    resetEncoders();

    m_algaeClosedLoopController = m_algaeArmMotor.getClosedLoopController();

    setArmState(ALGAE_ARM_STATE.RAISED);
  }

  //All get___AlgaeEncoder methods return value in radians
  public double getAlgaeTopEncoder() {
    return (m_algaeTopEncoder.getPosition() * Math.PI) / 21;
  }
  
  public double getAlgaeArmEncoder() {
    return (m_algaeArmEncoder.getPosition() * Math.PI) / 21;
  }

  public ALGAE_ARM_STATE getArmState(){
    return e_armState;
  }

  public void resetAlgaeEncoders() {
    m_algaeTopEncoder.setPosition(0); // reset the algae encoder ticks
    m_algaeArmEncoder.setPosition(0);
  }

  
  public void setArmState(ALGAE_ARM_STATE desiredState) { // setting the arm state; raised, centered,
    if (desiredState == ALGAE_ARM_STATE.RAISED) {
      d_desiredReferencePosition = Constants.AlgaeConstants.k_raisedArmPos;
    } else if (desiredState == ALGAE_ARM_STATE.CENTERED) {
      d_desiredReferencePosition = Constants.AlgaeConstants.k_centeredArmPos;
    } else if (desiredState == ALGAE_ARM_STATE.LOWERED) {
      d_desiredReferencePosition = Constants.AlgaeConstants.k_loweredArmPos;
    }
    e_armState = desiredState;
    m_algaeClosedLoopController.setReference(d_desiredReferencePosition, ControlType.kPosition); //make sure in radians
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

  public boolean isAtDesiredPosition(){
    return (Math.abs(getAlgaeArmEncoder() - d_desiredReferencePosition) < Constants.k_positionBuffer); //make sure in radians
  }

  // Dashboard methods
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("AlgaeTopEncoder in Degrees", getAlgaeTopEncoder() * 180 / Math.PI);
    SmartDashboard.putNumber("AlgaeArmEncoder in Degrees", getAlgaeArmEncoder() * 180 / Math.PI);
  }
  
  public void stop(){
    m_algaeTopMotor.stopMotor();
    m_algaeArmMotor.stopMotor();
  }

  //Still need to apply other methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(getAlgaeArmEncoder() - d_desiredReferencePosition > Constants.k_positionBuffer)
    //   m_algaeArmMotor.set(-Constants.AlgaeConstants.k_algaeArmSpeed);
    // else if (getAlgaeArmEncoder() - d_desiredReferencePosition < -Constants.k_positionBuffer)
    //   m_algaeArmMotor.set(Constants.AlgaeConstants.k_algaeArmSpeed);
    setSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}