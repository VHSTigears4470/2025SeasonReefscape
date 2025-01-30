package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.KitbotCoralConstants;

public class KitbotCoralSubsystem extends SubsystemBase{
 /** Variables for intake motors */
  private final PWMVictorSPX m_outputMotor;
  public KitbotCoralSubsystem() {
    m_outputMotor = new PWMVictorSPX(KitbotCoralConstants.k_outputMotorID);
  }

  public void intake() {
    m_outputMotor.setVoltage(-KitbotCoralConstants.k_fastVoltage);
  }
  public void stop(){
    m_outputMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(getArmEncoder() - d_desiredReferencePosition > Constants.k_positionBuffer)
    //   m_armMotor.set(-Constants.CoralConstants.k_coralArmSpeed);
    // else if (getArmEncoder() - d_desiredReferencePosition < -Constants.k_positionBuffer)
    //   m_armMotor.set(Constants.CoralConstants.k_coralArmSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}