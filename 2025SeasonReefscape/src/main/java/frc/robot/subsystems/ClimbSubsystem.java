// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

/** CLimb Subsystem */
public class ClimbSubsystem extends SubsystemBase {
  /** CLimb Subsystem */
   private final SparkMax m_climbMotor; 
   private final RelativeEncoder m_climbEncoder;
   private double d_desiredReferencePosition;
   private double d_upperLimit = 1;//change later to actual value
   private double d_lowerLimit = -1;//subject to change

   // Constructor:
  public ClimbSubsystem() {
    m_climbMotor = new SparkMax(Constants.ClimbConstants.k_climbMotorID, MotorType.kBrushless);
    m_climbEncoder = m_climbMotor.getEncoder();
    m_climbMotor.configure(Configs.MAXSwerveModule.climbMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    resetEncoders();
  }
    
  //All get___Encoder methods return value in radians
  public double getClimbEncoder(){
    return (m_climbEncoder.getPosition() * Math.PI) / 21;
  }

  public double getDesiredPos(){
    return d_desiredReferencePosition;
  }

  //If the arm is within its range of movement, adjusts its speed to the paramenter, otherwise makes the arm stop moving
  public void setArmSpeed(double speed){
      if((speed > 0 && getClimbEncoder() >= d_upperLimit) || (speed < 0 && getClimbEncoder() <= d_lowerLimit)){
        speed = 0;
      }
      m_climbMotor.set(speed);
  }

  //Sets the current position 0
  public void resetEncoders(){
    m_climbEncoder.setPosition(0);
  }

  //
  public boolean isAtDesiredPosition(){
    return (Math.abs(getClimbEncoder() - d_desiredReferencePosition) < Constants.k_positionBuffer);
  }

  // Dashboard Methods
  public void setSmartDashboard() {
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