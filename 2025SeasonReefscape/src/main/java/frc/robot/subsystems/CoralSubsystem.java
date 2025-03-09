// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Configs.CoralConfigs;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.LimitSwitchConstants;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;

public class CoralSubsystem extends SubsystemBase {
  /** Variables for intake motors */
  private final SparkMax m_intake;
  private final SparkMax m_armMotor;
  private final SparkClosedLoopController m_armMotorLoopController;

  private boolean m_hasHitStartLimit;
  private boolean m_hasHitEndLimit;

  private final DigitalInput m_startLimitSwitch;
  private final DigitalInput m_endLimitSwitch;
  private final RelativeEncoder m_armEncoder;
  private double d_desiredReferencePosition;
  private CORAL_ARM_STATE e_armState;
  //add sensor if necessary

  public CoralSubsystem() {
    m_intake = new SparkMax(CoralConstants.k_intakeID, MotorType.kBrushless);
    m_armMotor = new SparkMax(CoralConstants.k_armID, MotorType.kBrushless);
    m_armMotorLoopController = m_armMotor.getClosedLoopController();
    m_armEncoder = m_armMotor.getEncoder();

    m_hasHitStartLimit = false;
    m_hasHitEndLimit = false;

    if(LimitSwitchConstants.k_usingCoralStartLimitSwitch) {
      m_startLimitSwitch = new DigitalInput(CoralConstants.k_startLimitSwitchID);
    } else {
      m_startLimitSwitch = null;
    }
    if(LimitSwitchConstants.k_usingCoralSEndLimitSwitch) {
      m_endLimitSwitch = new DigitalInput(CoralConstants.k_endLimitSwitchID);
    } else {
      m_endLimitSwitch = null;
    }
    
    m_intake.configure(Configs.CoralConfigs.coralIntakeMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_armMotor.configure(Configs.CoralConfigs.coralArmMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    m_armEncoder.setPosition(CoralConstants.k_startingArmPos);
    setArmState(CoralConstants.CORAL_ARM_STATE.FORWARD);
  }

  /**
   * Gets the arm encoder
   * @return Arm encoder in radians
   */
  public double getArmEncoder() {
    return CoralConstants.k_coralArmEncoderReversed * m_armEncoder.getPosition();
  }


  /**
   * Resets the arm encoder to 0
   */
  public void resetEncoders(){
    m_armEncoder.setPosition(0);
  }

  /**
   * Gets the arm state that the robot has been set to 
   * (DOES NOT MEAN THAT THE ARM IS AT THAT POSITION, USE isAtDesiredPosition FOR THAT)
   * @return Arm State of the robot
   */
  public CORAL_ARM_STATE getArmState() {
    return e_armState;
  }

  /**
   * Shoots the coral out slow
   */
  public void shootSlow() {
    m_intake.setVoltage(CoralConstants.k_slowVoltage);
  }


  /**
   * Shoots the coral out fast
   */
  public void shootFast() {
    m_intake.setVoltage(CoralConstants.k_fastVoltage);
  }

  /**
   * Intake Coral
   */
  public void intake() {
    m_intake.setVoltage(CoralConstants.k_intakeVoltage);
  }

  /**
   * Sets the desired position of the coral arm
   * @param desiredState Position that the coral arm should be in
   */
  public void setArmState(CORAL_ARM_STATE desiredState) { //changes the position of the arm according to its desired state
    if (desiredState == CORAL_ARM_STATE.FORWARD) {
      d_desiredReferencePosition = Constants.CoralConstants.k_forwardArmPos;
    } else if (desiredState == CORAL_ARM_STATE.BACKWARD) {
      d_desiredReferencePosition = Constants.CoralConstants.k_backwardArmPos;
    }
    e_armState = desiredState;
    m_armMotorLoopController.setReference(d_desiredReferencePosition, ControlType.kMAXMotionPositionControl);
  }

  public void setTestReference(double position) {
    m_armMotorLoopController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Testing function to move the arm  motor
   * @param voltage of the arm motor
   */
  public void testArmMotorsVoltage(double voltage) {
    // if(getStartLimitSwitch()) {
    //   // If arm at climber side, only allow to go positive / climber side
    //   voltage = Math.min(voltage, 0);
    // }
    // if(getEndLimitSwitch()) {
    //   // If arm at algae side, only allow to go negative / algae side
    //   voltage = Math.max(voltage, 0);
    // }
    // Clamps the motor from -12 to 12
    // m_armMotor.setVoltage(Math.max(Math.min(voltage, 12), -12));
  }

  /**
   * Testing function to move the intake motor
   * @param speed of the intake motor
   */
  public void testIntakeMotorsVoltage(double voltage) {
    // Clamps the motor from -1 to 1
    m_intake.setVoltage(Math.max(Math.min(voltage, 12), -12));
  }

  /**
   * Function to determine if the arm is at the correct position
   * @return true if the robot is at the desired position
   */
  public boolean isAtDesiredPosition(){
    return (Math.abs(getArmEncoder() - d_desiredReferencePosition) < CoralConstants.k_positionBufferCoral);
  } 


  /**
   * Displays Values onto SmartDashboard (Encoder & Limit Switches)
   */
  public void setSmartDashboard() {
    SmartDashboard.putNumber("Coral Arm Encoder (Radians)", getArmEncoder());
    SmartDashboard.putNumber("Coral Arm Speed", m_armMotor.get());
    SmartDashboard.putBoolean("Coral Start Limit Switch", getStartLimitSwitch());
    SmartDashboard.putBoolean("Coral End Limit Switch", getEndLimitSwitch());
  }

  /**
   * Stops both the intake and arm motor
   */
  public void stopAllMotors(){
    m_intake.stopMotor();
    m_armMotor.stopMotor();
  }

  /**
   * Stops only the intake motor
   */
  public void stopIntakeMotor(){
    m_intake.stopMotor();
  }

  /**
   * Stops only the arm motor
   */
  public void stopArmMotor(){
    m_armMotor.stopMotor();
  }
  
  /**
   * Gets whether or not the start limit switch is hit
   * @return True when the limit switch is hit
   */
  public boolean getStartLimitSwitch() {
    if(LimitSwitchConstants.k_usingCoralStartLimitSwitch) {
      return !m_startLimitSwitch.get();
    } else {
      return false;
    }
  }

  /**
   * Gets whether or not the end limit switch is hit
   * @return True when the limit switch is hit
   */
  public boolean getEndLimitSwitch() {
    if(LimitSwitchConstants.k_usingCoralSEndLimitSwitch) {
      return m_endLimitSwitch.get();
    } else {
      return false;
    }
  }
  
  public void limitSwitchLogic() {

    if(getStartLimitSwitch()) {
      if(!m_hasHitStartLimit) {
        // m_armEncoder.setPosition(LimitSwitchConstants.k_coralStartLimitSwitchPosition);
        SmartDashboard.putString("Coral Sub PID", "Output Range Now : -0.5, 0.0");
        CoralConfigs.coralArmMotor.closedLoop.outputRange(CoralConstants.k_minArmOutput, 0.0);
        m_armMotor.configure(CoralConfigs.coralArmMotor, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_hasHitStartLimit = true;
      }
      m_hasHitEndLimit = false;
    } else if(getEndLimitSwitch()) {
      // m_armEncoder.setPosition(LimitSwitchConstants.k_coralEndLimitSwitchPosition);
      if(!m_hasHitEndLimit) {
        SmartDashboard.putString("Coral Sub PID", "Output Range Now : 0.0, 0.5");
        CoralConfigs.coralArmMotor.closedLoop.outputRange(0.0, CoralConstants.k_maxArmOutput); 
        m_armMotor.configure(CoralConfigs.coralArmMotor, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_hasHitEndLimit = true;
      }
      m_hasHitStartLimit = false;
    } else {
      if(m_hasHitEndLimit) {
        SmartDashboard.putString("Coral Sub PID", "Output Range Now : -0.5, 0.5");
        CoralConfigs.coralArmMotor.closedLoop.outputRange(CoralConstants.k_minArmOutput, CoralConstants.k_maxArmOutput);
        m_armMotor.configure(CoralConfigs.coralArmMotor, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_hasHitEndLimit = false;
      } 
      if(m_hasHitStartLimit) {
        SmartDashboard.putString("Coral Sub PID", "Output Range Now : -0.5, 0.5");
        CoralConfigs.coralArmMotor.closedLoop.outputRange(CoralConstants.k_minArmOutput, CoralConstants.k_maxArmOutput
        );
        m_armMotor.configure(CoralConfigs.coralArmMotor, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_hasHitStartLimit = false;
      }
    }

  }
  
  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
    setSmartDashboard();
    limitSwitchLogic();

    /*
    double voltageOutput;
    
    // Calculates proper state that the arm should be in 0.02 seconds
    TrapezoidProfile.State currentState = new TrapezoidProfile.State(m_armEncoder.getPosition(), m_armEncoder.getVelocity());
    TrapezoidProfile.State desiredState = new TrapezoidProfile.State(d_desiredReferencePosition, 0);
    TrapezoidProfile.State calculatedState = CoralConstants.trapezoidProfile.calculate(0.02, currentState, desiredState);

    // Calculate the proper voltage output with PID and feedforward
    voltageOutput = CoralConstants.k_armPID.calculate(calculatedState.position, calculatedState.velocity);
    voltageOutput += CoralConstants.k_armFeedForward.calculate(calculatedState.position, calculatedState.velocity);
    

    if(getStartLimitSwitch()) { // If starting limit switch is toggled on / arm on climber side
      m_armEncoder.setPosition(CoralConstants.k_startResetPosition); // Resets Position
      voltageOutput = Math.min(voltageOutput, 0); // Only allows voltage to be negative aka go towards algae sub side
    } 
    // else if(getEndLimitSwitch()) { // If ending limit switch is toggled on / arm on algae side
    //   m_armEncoder.setPosition(CoralConstants.k_endResetPosition); // Resets Position
    //   voltageOutput = Math.max(voltageOutput, 0); // Only allows voltage to be positive aka go towards climber sub side
    // }
    
      // Sets the voltage with from [-12 to 12]
      voltageOutput = Math.max(Math.min(voltageOutput, 12), -12);

      SmartDashboard.putNumber("Coral Arm Voltage Output", voltageOutput);
      // m_armMotor.setVoltage(voltageOutput);
      */
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}