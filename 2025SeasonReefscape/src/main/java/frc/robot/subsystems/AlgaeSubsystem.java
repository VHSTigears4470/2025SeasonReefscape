// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.ALGAE_ARM_STATE;

/** Algae Subsystem */
public class AlgaeSubsystem extends SubsystemBase {
  /** Algae Subsystem */
   private final SparkMax m_algaeTopMotor; /* Variables for algae motors */
   private final SparkMax m_algaeArmMotor;
   private final RelativeEncoder m_algaeArmEncoder;
   private final SparkClosedLoopController m_algaeClosedLoopController;
   
   private ALGAE_ARM_STATE e_armState;
   private double d_desiredReferencePosition;


  public AlgaeSubsystem(){
    m_algaeTopMotor = new SparkMax(Constants.AlgaeConstants.k_algaeTopID, MotorType.kBrushless); // motor controller
    m_algaeArmMotor = new SparkMax(Constants.AlgaeConstants.k_algaeArmID, MotorType.kBrushless);

    m_algaeArmEncoder = m_algaeArmMotor.getEncoder();
    resetEncoders();

    m_algaeTopMotor.configure(Configs.MAXSwerveModule.algaeTopMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_algaeArmMotor.configure(Configs.MAXSwerveModule.algaeArmMotor, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_algaeClosedLoopController = m_algaeArmMotor.getClosedLoopController();

    setArmState(ALGAE_ARM_STATE.RAISED);
  }
  
  public double getAlgaeArmEncoder() {
    return m_algaeArmEncoder.getPosition(); //
  }

  public ALGAE_ARM_STATE getArmState(){
    return e_armState;
  }

  
  public void setArmState(ALGAE_ARM_STATE desiredState) { // setting the arm state; raised, centered,
    if (desiredState == ALGAE_ARM_STATE.RAISED) {
      d_desiredReferencePosition = Constants.AlgaeConstants.k_raisedArmPos;
    } else if (desiredState == ALGAE_ARM_STATE.CENTERED) {
      d_desiredReferencePosition = Constants.AlgaeConstants.k_centeredArmPos;
    } else /*if (desiredState == ALGAE_ARM_STATE.LOWERED)*/ {
      d_desiredReferencePosition = Constants.AlgaeConstants.k_loweredArmPos;
    }
    e_armState = desiredState;
    m_algaeClosedLoopController.setReference(d_desiredReferencePosition, ControlType.kPosition); //make sure in radians
  }
  
  // This method takes in the algae
  public void intake() {
    m_algaeTopMotor.setVoltage(AlgaeConstants.k_intakeVoltage);
  }


  public void dispense() {
    m_algaeTopMotor.setVoltage(AlgaeConstants.k_dispenseVoltage);
  }

  // Resets the encoders
  public void resetEncoders(){
    m_algaeArmEncoder.setPosition(0);
  }

  public boolean isAtDesiredPosition(){
    return (Math.abs(getAlgaeArmEncoder() - d_desiredReferencePosition) < Constants.k_positionBuffer); //make sure in radians
  }

  public void testArmMotors(double speed) {
    m_algaeArmMotor.set(speed);
  }

  // Dashboard methods
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("AlgaeArmEncoder in Degrees", getAlgaeArmEncoder() * 180 / Math.PI);
  }
  
  // This method stops the motors
  public void stopAllMotors(){
    m_algaeTopMotor.stopMotor();
    m_algaeArmMotor.stopMotor();
  }

  // This method stops the motors
  public void stopIntakeMotor(){
    m_algaeTopMotor.stopMotor();
  }

  // This method stops the motors
  public void stopArmMotor(){
    m_algaeArmMotor.stopMotor();
  }

  //Still need to apply other methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isAtDesiredPosition())
    {
      // Algae Hold Speed
      m_algaeArmMotor.set(Constants.AlgaeConstants.k_algaeArmHoldSpeed); 
    }
    else
    {
      if(getAlgaeArmEncoder() < d_desiredReferencePosition) {
        // If encoder value means we still need to increase our angle
        m_algaeArmMotor.set(Constants.AlgaeConstants.k_algaeArmSpeed);
      } else {
        // If encoder value means we still need to decrease our angle
        m_algaeArmMotor.set(-1 * Constants.AlgaeConstants.k_algaeArmSpeed);
      }
    }
    setSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}