// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.CORAL_ARM_STATE;

public class CoralSubsystem extends SubsystemBase {
  /** Variables for intake motors */
  private final SparkMax m_intake;
  private final SparkMax m_armMotor;
  private final DigitalInput m_startLimitSwitch;
  private final DigitalInput m_endLimitSwitch;
  private final RelativeEncoder m_armEncoder;
  private double d_desiredReferencePosition;
  private CORAL_ARM_STATE e_armState;
  //add sensor if necessary

  public CoralSubsystem() {
    m_intake = new SparkMax(CoralConstants.k_intakeID, MotorType.kBrushless);
    m_armMotor = new SparkMax(CoralConstants.k_armID, MotorType.kBrushless);
    m_armEncoder = m_armMotor.getEncoder();

    m_startLimitSwitch = new DigitalInput(CoralConstants.k_startLimitSwitchID);
    m_endLimitSwitch = new DigitalInput(CoralConstants.k_endLimitSwitchID);
    
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
  }

  /**
   * Testing function to move the arm motor
   * @param speed of the arm motor
   */
  public void testArmMotors(double speed) {
    m_armMotor.set(Math.max(Math.min(speed, 1), -1));
  }

  /**
   * Testing function to move the arm  motor
   * @param voltage of the arm motor
   */
  public void testArmMotorsVoltage(double voltage) {
    // Clamps the motor from -12 to 12
    m_armMotor.setVoltage(Math.max(Math.min(voltage, 12), -12));
  }

  /**
   * Testing function to move the intake motor
   * @param speed of the intake motor
   */
  public void testIntakeMotors(double speed) {
    // Clamps the motor from -1 to 1
    m_intake.set(Math.max(Math.min(speed, 1), -1));
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
    SmartDashboard.putBoolean("Coral Start Limit Switch", m_startLimitSwitch.get());
    SmartDashboard.putBoolean("Coral End Limit Switch", m_endLimitSwitch.get());
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
  
  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
    setSmartDashboard();

    double voltageOutput;
    // Calculates proper state to be in 0.02 seconds
    TrapezoidProfile.State currentState = new TrapezoidProfile.State(m_armEncoder.getPosition(), m_armEncoder.getVelocity());
    TrapezoidProfile.State desiredState = new TrapezoidProfile.State(d_desiredReferencePosition, 0);
    TrapezoidProfile.State calculatedState = CoralConstants.trapezoidProfile.calculate(0.02, currentState, desiredState);

    // Calculate the proper voltage output
    voltageOutput = CoralConstants.k_armPID.calculate(calculatedState.position, calculatedState.velocity);
    voltageOutput += CoralConstants.k_armFeedForward.calculate(calculatedState.position, calculatedState.velocity);
    

    if(m_startLimitSwitch.get()) { // If starting limit switch is toggled on

      // Reset Position
      m_armEncoder.setPosition(CoralConstants.k_startResetPosition);
      
      // If arm motor is going forwards and going to cause problems
      voltageOutput = Math.min(voltageOutput, 0);
    } else if(m_endLimitSwitch.get()) { // If ending limit switch is toggled on
      // Reset Position
      m_armEncoder.setPosition(CoralConstants.k_endResetPosition);
      
      // If arm motor is going backwards and going to cause problems
      voltageOutput = Math.max(voltageOutput, 0);
    }
    
      // Sets the voltage with from [-12 to 12]
      m_armMotor.setVoltage(Math.max(Math.min(voltageOutput, 12), -12));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}