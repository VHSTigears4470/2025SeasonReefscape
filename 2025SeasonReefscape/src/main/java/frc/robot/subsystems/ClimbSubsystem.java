// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.CLIMB_STATE;

public class ClimbSubsystem extends SubsystemBase {
  /** CLimb Subsystem */
   private final SparkMax m_climbMotor; 
   private final RelativeEncoder m_climbEncoder;
   private CLIMB_STATE e_climbState;
   private double d_desiredReferencePosition;

  public ClimbSubsystem() {
    m_climbMotor = new SparkMax(Constants.ClimbConstants.k_climbMotorID, MotorType.kBrushless);
    m_climbEncoder = m_climbMotor.getEncoder();
    resetEncoders();

    setClimbState(CLIMB_STATE.DOWN);
  }

    
  //All get___Encoder methods return value in radians
  public double getClimbEncoder(){
    return (m_climbEncoder.getPosition() * Math.PI) / 21;
  }

  public CLIMB_STATE getClimbState(){
    return e_climbState;
  }

  public double getDesiredPos(){
    return d_desiredReferencePosition;
  }

  public void setClimbState(CLIMB_STATE p_desiredState) {
    if (p_desiredState == CLIMB_STATE.UP) {
      e_climbState = p_desiredState;
      d_desiredReferencePosition = ClimbConstants.k_upClimbPos;
    } else if (p_desiredState == CLIMB_STATE.DOWN) {
      e_climbState = p_desiredState;
      d_desiredReferencePosition = ClimbConstants.k_downClimbPos;
    }
    //pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
    //Apply above in config
  }

  public void resetEncoders(){
    m_climbEncoder.setPosition(0);
  }

  public void setSmartDashboard(){
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("ClimbEncoder", getClimbEncoder() * 180 / Math.PI);
  }

  public void stop(){
    m_climbMotor.stopMotor();
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
