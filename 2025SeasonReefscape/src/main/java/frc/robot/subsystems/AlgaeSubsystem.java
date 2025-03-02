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

import edu.wpi.first.wpilibj.DigitalInput;
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
   private final DigitalInput m_downLimitSwitch;

   private final SparkClosedLoopController m_algaeClosedLoopController;
   
   private ALGAE_ARM_STATE e_armState;
   private double d_desiredReferencePosition;
   private boolean b_stowArmWhenIdle;


  public AlgaeSubsystem(){
    m_algaeIntakeMotor = new SparkMax(Constants.AlgaeConstants.k_algaeTopID, MotorType.kBrushless); // motor controller
    m_algaeArmMotor = new SparkMax(Constants.AlgaeConstants.k_algaeArmID, MotorType.kBrushless);
    m_algaeArmEncoder = m_algaeArmMotor.getEncoder();

    m_downLimitSwitch = new DigitalInput(AlgaeConstants.k_downLimitSwitchID);

    m_algaeIntakeMotor.configure(Configs.AlgaeConfigs.algaeIntakeMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_algaeArmMotor.configure(Configs.AlgaeConfigs.algaeArmMotor, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_algaeClosedLoopController = m_algaeArmMotor.getClosedLoopController();

    b_stowArmWhenIdle = true;
    m_algaeArmEncoder.setPosition(0); // 'DOWN' is at 0
  }
  
  /**
   * Gets the algae arm encoder
   * @return algae arm encoder
   */
  public double getAlgaeArmEncoder() {
    return AlgaeConstants.k_algaeArmEncoderReversed * m_algaeArmEncoder.getPosition(); 
  }

  /**
   * Returns the desired arm state
   * @return The current desired arm state
   */
  public ALGAE_ARM_STATE getArmState(){
    return e_armState;
  }

  /**
   * Sets the arm state 
   * (DOES NOT MEAN THE ARM IS ALREADY THERE, IT JUST MEANS THAT IT IS GOING THERE)
   * @param desiredState
   */
  public void setArmState(ALGAE_ARM_STATE desiredState) { // setting the arm state; DOWN, HOLDING, STOWED
    e_armState = desiredState;
    if (desiredState == ALGAE_ARM_STATE.DOWN) 
      d_desiredReferencePosition = Constants.AlgaeConstants.k_downArmPos;
    else if (desiredState == ALGAE_ARM_STATE.HOLDING) 
      d_desiredReferencePosition = Constants.AlgaeConstants.k_holdingArmPos;
    else  // ALGAE_ARM_STATE.STOWED
      d_desiredReferencePosition = Constants.AlgaeConstants.k_stowedArmPos;
    
    m_algaeClosedLoopController.setReference(d_desiredReferencePosition, ControlType.kPosition); //make sure in radians
  }
  
  /**
   * Sets the voltage for the intake motor to intake the algae
   */
  public void intake() {
    m_algaeIntakeMotor.setVoltage(AlgaeConstants.k_intakeVoltage); 
  }

  
  /**
   * This methods
   */
  public void dispense() {
    m_algaeIntakeMotor.setVoltage(AlgaeConstants.k_dispenseVoltage);
  }

  /**
   * Sets voltage for intake motor to hold the alage
   */
  public void hold() {
    m_algaeIntakeMotor.setVoltage(AlgaeConstants.k_holdVoltage);
  }

  /**
   * Resets the arm encoders to 0
   */
  public void resetEncoders(){
    m_algaeArmEncoder.setPosition(0);
  }

  /**
   * Sets the speed of the arm motor
   * @param speed of the arm motor
   */
  public void testArmMotors(double speed) {
    m_algaeArmMotor.set(speed);
  }

  /**
   * Sets the speed of the intake
   * @param speed of the intake motor
   */
  public void testIntakeMotors(double speed) {
    m_algaeIntakeMotor.set(speed);
  }

  /**
   * Sets if the arm should be in stow while idle
   * @param stowWhenIdle True is the robot should be in stow when idle
   */
  public void setStowArmWhenIdle(boolean stowWhenIdle) {
    b_stowArmWhenIdle = stowWhenIdle;
  }

  /**
   * Whether the robot is commanded to stow arm when idle
   * @return Whether the robot is commanded to stow arm when idle
   */
  public boolean getStowArmWhenIdle() {
    return b_stowArmWhenIdle;
  }

  /**
   * Sets the SmartDashboard for the Algae Arm 
   */
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("Algae Arm Encoder (Radians)", getAlgaeArmEncoder());
    SmartDashboard.putBoolean("Algae Down Limit Switch", m_downLimitSwitch.get());
  }
  
  /**
   * Stops both the arm and intake motor
   */
  public void stopAllMotors(){
    m_algaeIntakeMotor.stopMotor();
    m_algaeArmMotor.stopMotor();
  }

  /**
   * Stops only the intake motor
   */
  public void stopIntakeMotor(){
    m_algaeIntakeMotor.stopMotor();
  }

  /**
   * Stops only the arm motor
   */
  public void stopArmMotor(){
    m_algaeArmMotor.stopMotor();
  }

@Override
  public void periodic() {
    setSmartDashboard();
    if(m_downLimitSwitch.get()) {
      if(m_algaeArmMotor.getBusVoltage() >= 0) { // If hitting the down limit switch and voltage is wants to go further
        m_algaeArmMotor.setVoltage(0);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}