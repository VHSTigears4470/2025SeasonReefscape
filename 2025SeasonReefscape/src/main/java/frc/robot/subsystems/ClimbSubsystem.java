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
import frc.robot.Constants.ClimbConstants.CLIMB_STATE;

/** CLimb Subsystem */
public class ClimbSubsystem extends SubsystemBase {
  /** Variables for intake motors */
   private final SparkMax m_climbMotorSparkMax; 
   private final RelativeEncoder m_climbEncoder;
   private CLIMB_STATE e_climbState;

   // Constructor:
  public ClimbSubsystem() {
    m_climbMotorSparkMax = new SparkMax(Constants.ClimbConstants.k_climbMotorID, MotorType.kBrushless);
    m_climbEncoder = m_climbMotorSparkMax.getEncoder();
  }

  //All get___ClimbEncoder methods return value in radians
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

  // Dashboard Methods
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("ClimbEncoder", getClimbEncoder() * 180 / Math.PI);
  }

  public void resetAlgaeEncoder() {
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
