package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class UnjamIntakeForTime extends CommandBase {
    private Intake m_intake;
    private Hopper m_hopper;
    private Flywheel m_flywheel;
    private double initialTime, totalTime;

    public UnjamIntakeForTime(double time) {
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        m_flywheel = Flywheel.getInstance();
        addRequirements(m_intake, m_hopper, m_flywheel);

        totalTime = time;
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();

        m_intake.reverseIntake(Constants.INTAKE_SPEED);
        m_hopper.reverseHopper(Constants.HOPPER_INDEX_POWER);
    }

    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
        m_hopper.stopHopper();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if((Timer.getFPGATimestamp() - initialTime) > totalTime){
            return true;
        }
        return false;
    }
}