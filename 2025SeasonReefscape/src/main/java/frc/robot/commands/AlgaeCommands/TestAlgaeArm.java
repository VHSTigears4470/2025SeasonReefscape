package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class TestAlgaeArm extends Command {
    private AlgaeSubsystem m_algaeSub;
    private final double m_speed;
        
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public TestAlgaeArm(AlgaeSubsystem algaeSub, double speed) {
        m_algaeSub = algaeSub;
        m_speed = speed;
        addRequirements(m_algaeSub);
    }

    // Called when the command is initially scheduled.
     @Override
     public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        m_algaeSub.testArmMotors(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        m_algaeSub.stopArmMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }
}