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
  private final RelativeEncoder m_armEncoder;
  private double d_desiredReferencePosition;
  private CORAL_ARM_STATE e_armState;
  //add sensor if necessary

  public CoralSubsystem() {
    m_intake = new SparkMax(CoralConstants.k_botID, MotorType.kBrushless);
    m_armMotor = new SparkMax(CoralConstants.k_armID, MotorType.kBrushless);
    m_armEncoder = m_armMotor.getEncoder();
    
    m_intake.configure(Configs.MAXSwerveModule.intakeMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_armMotor.configure(Configs.MAXSwerveModule.intakeMotorArm, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    m_armEncoder.setPosition(CoralConstants.k_startingArmPos);
    setArmState(CORAL_ARM_STATE.FORWARD);
  }

  //All get___Encoder methods return ____Encoder values in radians
//Returns the position of the arm encoder
  public double getArmEncoder() {
    return m_armEncoder.getPosition();
  }

//Resets the encoder
  public void resetEncoders(){
    m_armEncoder.setPosition(0);
  }

//Returning the arm state
  public CORAL_ARM_STATE getArmState() {
    return e_armState;
  }

//Shoots slow by decreasing the voltage of the top and bottom motors
  public void shootSlow() {
    m_intake.setVoltage(CoralConstants.k_slowVoltage);
  }

//Shoots fast by increasing the voltage of the top and bottom motors
  public void shootFast() {
    m_intake.setVoltage(CoralConstants.k_fastVoltage);
  }

  
  public void setArmState(CORAL_ARM_STATE desiredState) { //changes the position of the arm according to its desired state
    if (desiredState == CORAL_ARM_STATE.FORWARD) {
      d_desiredReferencePosition = Constants.CoralConstants.k_forwardArmPos;
    } else if (desiredState == CORAL_ARM_STATE.BACKWARD) {
      d_desiredReferencePosition = Constants.CoralConstants.k_backwardArmPos;
    }
    e_armState = desiredState;
  }

  public void testArmMotors(double speed) {
    m_armMotor.set(speed);
  }

   public boolean isAtDesiredPosition(){
    return (Math.abs(getArmEncoder() - d_desiredReferencePosition) < CoralConstants.k_positionBufferCoral);
  } 

//sets the encoders to default values
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 

    SmartDashboard.putNumber("ArmEncoder (Degrees)", getArmEncoder() * 180 / Math.PI);
  }

  //stops all the motors
  public void stopAllMotors(){
    m_intake.stopMotor();
    m_armMotor.stopMotor();
  }

  //stops the motors
  public void stopIntakeMotor(){
    m_intake.stopMotor();
  }

  //stops the motors
  public void stopArmMotor(){
    m_armMotor.stopMotor();
  }
  
  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
    setSmartDashboard();

    TrapezoidProfile.State currentState = new TrapezoidProfile.State(m_armEncoder.getPosition(), m_armEncoder.getVelocity());
    TrapezoidProfile.State desiredState = new TrapezoidProfile.State(d_desiredReferencePosition, 0);
    TrapezoidProfile.State calculatedState = CoralConstants.trapezoidProfile.calculate(0.02, currentState, desiredState);

    double voltageOutput = CoralConstants.k_armPID.calculate(calculatedState.position, calculatedState.velocity);
    voltageOutput += CoralConstants.k_armFeedForward.calculate(calculatedState.position, calculatedState.velocity);
    
    m_armMotor.setVoltage(voltageOutput);   
   }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}