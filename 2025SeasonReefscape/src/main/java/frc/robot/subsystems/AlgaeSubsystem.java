// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.LimitSwitchConstants;

/** Algae Subsystem */
public class AlgaeSubsystem extends SubsystemBase {
  /** Algae Subsystem */
   private final SparkMax m_algaeIntakeMotor; /* Variables for algae motors */
   private final SparkMax m_algaeArmMotor;
   private final RelativeEncoder m_algaeArmEncoder;
   private final DigitalInput m_downLimitSwitch;
   private final DigitalInput m_upLimitSwitch;

  public AlgaeSubsystem(){
    m_algaeIntakeMotor = new SparkMax(Constants.AlgaeConstants.k_algaeTopID, MotorType.kBrushless); // motor controller
    m_algaeArmMotor = new SparkMax(Constants.AlgaeConstants.k_algaeArmID, MotorType.kBrushless);
    m_algaeArmEncoder = m_algaeArmMotor.getEncoder();

    if(LimitSwitchConstants.k_usingAlgaeDownLimitSwitch) {
      m_downLimitSwitch = new DigitalInput(AlgaeConstants.k_downLimitSwitchID);
    } else {
      m_downLimitSwitch = null;
    }

    if(LimitSwitchConstants.k_usingAlgaeUpLimitSwitch) {
      m_upLimitSwitch = new DigitalInput(AlgaeConstants.k_upLimitSwitchID);
    } else {
      m_upLimitSwitch = null;
    }

    m_algaeIntakeMotor.configure(Configs.AlgaeConfigs.algaeIntakeMotor, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_algaeArmMotor.configure(Configs.AlgaeConfigs.algaeArmMotor, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_algaeArmEncoder.setPosition(0); 
  }
  
  /*
   * Gets the algae arm encoder
   * @return algae arm encoder
   */
  public double getAlgaeArmEncoder() {
    return AlgaeConstants.k_algaeArmEncoderReversed * m_algaeArmEncoder.getPosition(); 
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

  public void armForward() {
    m_algaeArmMotor.setVoltage(Constants.AlgaeConstants.k_forwardVoltage);
  }

  public void holdIdle() {
    m_algaeArmMotor.setVoltage(Constants.AlgaeConstants.k_idleVoltage);
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
    if(getDownLimitSwitch()) {
      // if already at down max, only allows negative / going up
      speed = Math.min(speed, 0);
    }
    if(getUpLimitSwitch()) {
      // if already at up max, only allows going down / positive
      speed = Math.max(speed, 0);
    }
    m_algaeArmMotor.set(speed);
  }

  /**
   * Sets the voltage of the intake
   * @param voltage of the intake motor
   */
  public void testIntakeMotorsVoltage(double voltage) {
    m_algaeIntakeMotor.setVoltage(voltage);
  }

  /**
   * Sets the SmartDashboard for the Algae Arm 
   */
  public void setSmartDashboard() {
    //Encoder values in degrees - subject to change 
    SmartDashboard.putNumber("Algae Arm Encoder (Radians)", getAlgaeArmEncoder());
    SmartDashboard.putNumber("Algae Arm Bus Voltage", m_algaeArmMotor.getBusVoltage());
    SmartDashboard.putBoolean("Algae Down Limit Switch", getDownLimitSwitch());
    SmartDashboard.putBoolean("Algae Up Limit Switch", getUpLimitSwitch());
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

  /**
   * Gets whether or not the down limit switch is hit
   * @return True when the limit switch is hit
   */
  public boolean getDownLimitSwitch() {
    if(LimitSwitchConstants.k_usingAlgaeDownLimitSwitch) {
      return m_downLimitSwitch.get();
    } else {
      return false;
    }
  }

  /**
   * Gets whether or not the up limit switch is hit
   * @return True when the limit switch is hit
   */
  public boolean getUpLimitSwitch() {
    if(LimitSwitchConstants.k_usingAlgaeUpLimitSwitch) {
      return m_upLimitSwitch.get();
    } else {
      return false;
    }
  }

@Override
  public void periodic() {
    setSmartDashboard();
    if(getDownLimitSwitch()) {
      // Prevents Algae Arm From Going Down Further
      if(m_algaeArmMotor.getBusVoltage() >= 0) {
        m_algaeArmMotor.setVoltage(0);
      }
    }
    if(getUpLimitSwitch()) {
      // Prevents Algae Arm From Going Up Further
      if(m_algaeArmMotor.getBusVoltage() <= 0) { 
        m_algaeArmMotor.setVoltage(0);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}