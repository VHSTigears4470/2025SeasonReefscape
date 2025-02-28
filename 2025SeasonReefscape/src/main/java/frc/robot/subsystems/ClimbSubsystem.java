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
import frc.robot.Constants.ClimbConstants;

/** CLimb Subsystem */
public class ClimbSubsystem extends SubsystemBase {
  /** CLimb Subsystem */
   private final SparkMax m_climbMotor; 
   private final RelativeEncoder m_climbEncoder;
   private double d_desiredReferencePosition;
   private double d_pullingUpLimit = 1;//change later to actual value
   private double d_loweringLimit = -1;//subject to change

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
    return ClimbConstants.k_climbEncoderReversed * m_climbEncoder.getPosition();
  }

  public double getDesiredPos(){
    return d_desiredReferencePosition;
  }

  //If the arm is within its range of movement, adjusts its speed to the paramenter, otherwise makes the arm stop moving
  public void setArmSpeed(double speed){
      if((speed > 0 && getClimbEncoder() >= d_pullingUpLimit) || (speed < 0 && getClimbEncoder() <= d_loweringLimit)){
        speed = 0;
      }
      m_climbMotor.set(speed);
  }

  /**
   * Ignores upper and lower limits and sets speed
   * Should ONLY be used for calibrating robot
   * @param speed set motors to
   */
  public void setArmSpeedOverride(double speed) {
    m_climbMotor.set(speed);
  }

  //Sets the current position 0
  public void resetEncoders(){
    m_climbEncoder.setPosition(0);
  }

  //
  public boolean isAtDesiredPosition(){
    return (Math.abs(getClimbEncoder() - d_desiredReferencePosition) < ClimbConstants.k_positionBufferClimb);
  }

  // Dashboard Methods
  public void setSmartDashboard() {
    SmartDashboard.putNumber("Climb Encoder (Radians)", getClimbEncoder());
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