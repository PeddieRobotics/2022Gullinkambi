// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase{
    
  private Drivetrain drivetrain;
  private OI oi;

  public Drive() {
      drivetrain = Drivetrain.getInstance();
      oi = OI.getInstance();

      addRequirements(drivetrain);
  }
  
  // Initializes the drive command
  @Override
  public void initialize(){}

  // Executes the drive command
  @Override
  public void execute(){
      double speedInput = oi.getSpeed();
      double turnInput = oi.getTurn();

      drivetrain.arcadeDrive(speedInput, turnInput);
  }

  // End the command if it is interrupted
  @Override
  public void end(boolean interrupted){}

  // Checks if the command is finished
  @Override
  public boolean isFinished(){
      return false;
  }
}