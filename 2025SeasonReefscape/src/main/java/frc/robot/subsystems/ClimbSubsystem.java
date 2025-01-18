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
  /** Creates a new ExampleSubsystem. */
   private final SparkMax m_climbMotoSparkMax; 
   private final RelativeEncoder m_climbEncoder;

  public ClimbSubsystem() {
    m_climbMotoSparkMax = new SparkMax(Constants.ClimbConstants.k_climbMotorID, MotorType.kBrushless);
    m_climbEncoder = m_climbMotoSparkMax.getEncoder();
  }

  public double getClimbEncoder(){
    return (m_climbEncoder.getPosition() * Math.PI) / 21;
  }

  //Still need to apply other methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
