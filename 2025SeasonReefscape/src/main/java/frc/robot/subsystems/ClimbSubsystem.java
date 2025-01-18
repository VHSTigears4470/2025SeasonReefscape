// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  /** CLimb Subsystem */
   private final SparkMax m_algaeMotorSparkMax; 
   private final RelativeEncoder m_algaeEncoder;
   private CLIMB_STATE e_armState;

  public ClimbSubsystem() {
    m_algaeMotorSparkMax = new SparkMax(Constants.ClimbConstants.k_climbMotorID, MotorType.kBrushless);
    m_algaeEncoder = m_algaeMotorSparkMax.getEncoder();
  }

  public double getClimbEncoder(){
    return (m_climbEncoder.getPosition() * Math.PI) / 21;
  }

  public CLIMB_STATE getClimbState() {
    return e_climbState;
  }

  public void setClimbState(CLIMB_STATE desiredState) {
    if (desiredState == CLIMB_STATE.UP) {
      e_climbState = desiredState;
      //desiredReferencePosition = highestPos;
    } else if (desiredState == CLIMB_STATE.DOWN) {
      e_climbState = desiredState;
      //desiredReferencePosition = lowestPos;
    }
    //pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }

  public void setSmartDashboard(){
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("ClimbEncoder", getClimbEncoder() * 180 / Math.PI);
  }

  public void resetAlgaeEncoder(){
    m_climbEncoder.setPosition(0);
  }

  //Still need to apply other methods

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
