package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KitbotCoralConstants;

public class KitbotCoralSubsystem extends SubsystemBase{
 /** Variables for intake motors */
  private final PWMVictorSPX m_outputMotor;
  public KitbotCoralSubsystem() {
    m_outputMotor = new PWMVictorSPX(KitbotCoralConstants.k_outputMotorID);
  }

  public void intake() { //puts the motor in motion 
    m_outputMotor.setVoltage(-KitbotCoralConstants.k_fastVoltage);
  }
  
  // This stops the motor
  public void stop(){ 
    m_outputMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
  
  }

  
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}