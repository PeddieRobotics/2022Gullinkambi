package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoWaitFromDashboard extends CommandBase {

    private double initialTime;
    private double givenWaitTime;
    private String nameOfKey;

    public AutoWaitFromDashboard(String dashboardKey) {
        initialTime = 0;
        givenWaitTime = 0;
        nameOfKey = dashboardKey;
    
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        givenWaitTime = SmartDashboard.getNumber(nameOfKey, 0.0);
       
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
          return ((Timer.getFPGATimestamp() - initialTime) > givenWaitTime);
      }
    }