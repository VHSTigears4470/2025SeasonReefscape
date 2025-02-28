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
   private final SparkMax m_algaeIntakeMotor; /* Variables for algae motors */
   private final SparkMax m_algaeArmMotor;
   private final RelativeEncoder m_algaeArmEncoder;
   private final SparkClosedLoopController m_algaeClosedLoopController;
   
   private ALGAE_ARM_STATE e_armState;
   private double d_desiredReferencePosition;
   private boolean b_stowArmWhenIdle;


  public AlgaeSubsystem(){
    m_algaeIntakeMotor = new SparkMax(Constants.AlgaeConstants.k_algaeIntakeID, MotorType.kBrushless); // motor controller
    m_algaeArmMotor = new SparkMax(Constants.AlgaeConstants.k_algaeArmID, MotorType.kBrushless);

    m_algaeArmEncoder = m_algaeArmMotor.getEncoder();

    m_algaeIntakeMotor.configure(Configs.AlgaeConfigs.algaeIntakeMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_algaeArmMotor.configure(Configs.AlgaeConfigs.algaeArmMotor, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_algaeClosedLoopController = m_algaeArmMotor.getClosedLoopController();

    b_stowArmWhenIdle = true;
    setArmState(AlgaeConstants.k_startingArmState); // TODO
    m_algaeArmEncoder.setPosition(AlgaeConstants.k_startingArmPos); // TODO
  }
  
  public double getAlgaeArmEncoder() {
    return AlgaeConstants.k_algaeArmEncoderReversed * m_algaeArmEncoder.getPosition(); 
  }

  public ALGAE_ARM_STATE getArmState(){
    return e_armState;
  }

  public void setArmState(ALGAE_ARM_STATE desiredState) { // setting the arm state; raised, centered,
    if (desiredState == ALGAE_ARM_STATE.DOWN) 
      d_desiredReferencePosition = Constants.AlgaeConstants.k_downArmPos;
    else if (desiredState == ALGAE_ARM_STATE.HOLDING) 
      d_desiredReferencePosition = Constants.AlgaeConstants.k_holdingArmPos;
    else  // ALGAE_ARM_STATE.STOWED
      d_desiredReferencePosition = Constants.AlgaeConstants.k_stowedArmPos;
    
    e_armState = desiredState;
    m_algaeClosedLoopController.setReference(d_desiredReferencePosition, ControlType.kPosition); //make sure in radians
  }
  
  // This method takes in the algae
  public void intake() {
    m_algaeIntakeMotor.setVoltage(AlgaeConstants.k_intakeVoltage);
  }

  public void dispense() {
    m_algaeIntakeMotor.setVoltage(AlgaeConstants.k_dispenseVoltage);
  }

  public void hold() {
    m_algaeIntakeMotor.setVoltage(AlgaeConstants.k_holdVoltage);
  }

  // Resets the encoders
  public void resetEncoders(){
    m_algaeArmEncoder.setPosition(0);
  }

  public void testArmMotors(double speed) {
    m_algaeArmMotor.set(speed);
  }

  public void testIntakeMotors(double speed) {
    m_algaeIntakeMotor.set(speed);
  }

  public void setStowArmWhenIdle(boolean stowWhenIdle) {
    b_stowArmWhenIdle = stowWhenIdle;
  }

  public boolean getStowArmWhenIdle() {
    return b_stowArmWhenIdle;
  }

  // Dashboard methods
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("Algae Arm Encoder (Radians)", getAlgaeArmEncoder());
  }
  
  // This method stops the motors
  public void stopAllMotors(){
    m_algaeIntakeMotor.stopMotor();
    m_algaeArmMotor.stopMotor();
  }

  // This method stops the motors
  public void stopIntakeMotor(){
    m_algaeIntakeMotor.stopMotor();
  }

  // This method stops the motors
  public void stopArmMotor(){
    m_algaeArmMotor.stopMotor();
  }

@Override
  public void periodic() {
    setSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}