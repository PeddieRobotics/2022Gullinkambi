package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RollingAverage;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  
  private static Limelight limelight;
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry thor = limelightTable.getEntry("thor");
  NetworkTableEntry tvert = limelightTable.getEntry("tvert");
  NetworkTableEntry ta = limelightTable.getEntry("ta");
  NetworkTableEntry tv = limelightTable.getEntry("tv");
  private RollingAverage txAverage = new RollingAverage();
  private RollingAverage tyAverage = new RollingAverage();
    
  public Limelight() {
  }
   
  public static Limelight getInstance() {
    if (limelight == null) {
      limelight = new Limelight();
    }
  return limelight;

  }

  @Override
  public void periodic(){
    updateRollingAverages();
    SmartDashboard.putNumber("Limelight vertical error", getTy());
    SmartDashboard.putNumber("Limelight horizontal error", getTx());
    SmartDashboard.putNumber("Limelight distance", getDistance());
  }

  //Tvert is the vertical sidelength of the rough bounding box (0 - 320 pixels)
  public double getTvert(){
    return tvert.getDouble(0.0);
  }

  //Thor is the horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double getThor(){
    return thor.getDouble(0.0);
  }
  
  //Tx is the Horizontal Offset From Crosshair To Target
  public double getTx(){
    return tx.getDouble(0.0);
  }
  
  //Ty is the Vertical Offset From Crosshair To Target
  public double getTy(){
    return ty.getDouble(0.0);
  }
  
  public double getTa(){
    return ta.getDouble(0.0);
  }

  public double getTxAverage(){
    return txAverage.getAverage();
  }

  public double getTyAverage(){
    return tyAverage.getAverage();
  }
  
  public double getDistance(){
    if(ty.getDouble(0.0)==0){
      return 0;
    }
     else return (104-24)/(Math.tan(Math.toRadians(36+ty.getDouble(0.0))));
  }
  
  public boolean hasTarget(){
    if (limelightTable.getEntry("tv").getDouble(0.0)==1){
      return true;
    }else return false;
  }

  public void updateRollingAverages(){
    if(hasTarget()){
      txAverage.add(getTx());
      tyAverage.add(getTy());
    }
  }

  public void putSmartDashboardOverrides(){
  }
}