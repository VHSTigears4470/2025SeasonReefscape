// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
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
   private final DigitalInput m_maxLimitSwitch;

   // Constructor:
  public ClimbSubsystem() {
    m_climbMotor = new SparkMax(Constants.ClimbConstants.k_climbMotorID, MotorType.kBrushless);
    m_climbEncoder = m_climbMotor.getEncoder();
    m_maxLimitSwitch = new DigitalInput(ClimbConstants.k_maxLimitSwitchID);
    m_climbMotor.configure(Configs.ClimbConfigs.climbMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    resetEncoders();
  }
    
  /**
   * Gets the encoder for the climb
   * @return Gets teh encoder for the climb
   */
  public double getClimbEncoder(){
    return ClimbConstants.k_climbEncoderReversed * m_climbEncoder.getPosition();
  }

  /**
   * Sets the climb speed, will stop after it reaches a soft limit or a hard limit
   * @param speed
   */
  public void setArmSpeed(double speed){
    if(m_maxLimitSwitch.get() && speed > 0) {
      // robot hits the max limit and still wants to pull up
      speed = 0;
    }
    if(getClimbEncoder() >= ClimbConstants.k_pullUpClimbPos && speed > 0) { 
      // robot already at pulling up soft limit and still wants to pull up
      speed = 0;
    }
    if(getClimbEncoder() <= ClimbConstants.k_releaseDownClimbPos && speed < 0){
      // robot already at release soft limit and still wants to go down
      speed = 0;    
    }

    m_climbMotor.set(speed);
  }

  /**
   * Ignores upper and lower soft limits and sets speed
   * Hard limits still in place
   * Should ONLY be used for calibrating robot
   * @param speed set motors to
   */
  public void setArmSpeedOverride(double speed) {
    if(m_maxLimitSwitch.get() && speed > 0) {
      // robot already at max and still wants to pull up
      speed = 0;
    }
    m_climbMotor.set(speed);
  }

  /**
   * Resets the climb encoders 0
   */
  public void resetEncoders(){
    m_climbEncoder.setPosition(0);
  }

  /**
   * Puts values into SmartDashboard (Encoder)
   */
  public void setSmartDashboard() {
    //Encoder values in radians - subject to change 
    SmartDashboard.putNumber("Climb Encoder (Radians)", getClimbEncoder());
  }
  
  /** 
   * Stops the climb motor
   */  
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