package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{

    private static Lights lights;
    private Solenoid strip;

    public Lights() {
        strip = new Solenoid(21,PneumaticsModuleType.REVPH, 15);
    }

    public static Lights getInstance(){
        if(lights == null){
          lights = new Lights();
        }
        return lights;
      }
  
    public void on(){
        strip.set(true);
    }  

    public void off(){
        strip.set(false);
    }  

    public void timeCheck(){
      if (SmartDashboard.getNumber("Remaining Match Time", Timer.getMatchTime()) <= 15 && SmartDashboard.getNumber("Remaining Match Time", Timer.getMatchTime()) > 10){
        on();
        off();
      }
    }
    @Override
    public void periodic() {
      timeCheck();
     }
}