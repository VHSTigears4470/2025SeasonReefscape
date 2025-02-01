package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KitbotCoralConstants;

public class KitbotCoralSubsystem extends SubsystemBase{
 /** Variables for intake motors */
 // To Change VictorSPX ID, go to Pheonix Tuner X => Settings => FRC Advanced => Run Temporary Diagonstic Server => Go Back To Device
 // How To Hardware CRF Update => Go To https://github.com/CrossTheRoadElec/Phoenix-Releases => Search Up "CRF" and click correct file and download
  private final VictorSPX m_outputMotor;
  public KitbotCoralSubsystem() {
    m_outputMotor = new VictorSPX(KitbotCoralConstants.k_outputMotorID);
  }

  public void intake() { //puts the motor in motion 
    m_outputMotor.set(VictorSPXControlMode.PercentOutput, -KitbotCoralConstants.k_speed);
  }
  
  public void setSpeed(double speed) {
    m_outputMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }
  
  // This stops the motor
  public void stop(){ 
    m_outputMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
  
  @Override
  public void periodic() {
  
  }

  
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}