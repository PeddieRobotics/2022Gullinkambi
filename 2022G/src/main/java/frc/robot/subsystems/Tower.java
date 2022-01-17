package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class Tower extends SubsystemBase{

    private CANSparkMax towerBeltUpper, towerBeltLower;
    private static Tower tower;
    //private AnalogInput topSensor0, middleSensor1, bottomSensor2, bottomSensor3;
    private DigitalInput topSensor0, middleSensor1, bottomSensor2, bottomSensor3;
    
    public static Tower getInstance(){
        if (tower == null) {
            tower = new Tower();
        }
        return tower;
    }
    
    public Tower() {
        setupTower();
        
        topSensor0 = new DigitalInput(0);
        middleSensor1 = new DigitalInput(1);
        bottomSensor2 = new DigitalInput(2);
        bottomSensor3 = new DigitalInput(3);

        /* 
        topSensor0 = new AnalogInput(0);
        middleSensor1 = new AnalogInput(1);
        bottomSensor2 = new AnalogInput(2);
        bottomSensor3 = new AnalogInput(3);
        */
    }

    private void setupTower() {
        towerBeltUpper = new CANSparkMax(RobotMap.TOWER_BELT_UPPER, MotorType.kBrushless);
        towerBeltLower = new CANSparkMax(RobotMap.TOWER_BELT_LOWER, MotorType.kBrushless);

        towerBeltUpper.setIdleMode(IdleMode.kBrake);
        towerBeltLower.setIdleMode(IdleMode.kBrake);

        towerBeltUpper.setSmartCurrentLimit(Constants.MAX_TOWERBELTS_SPEED);
        towerBeltLower.setSmartCurrentLimit(Constants.MAX_TOWERBELTS_SPEED);
    }

    public void reverseTower(double percent) { 
        runLowerBelt(-percent);
        runUpperBelt(-percent);
    }

    public void runLowerBelt(double speed){
        towerBeltLower.set(speed);
    }

    public void runUpperBelt(double speed){ 
        towerBeltUpper.set(speed);
    }

    public void runTowerBelts(double upperSpeed, double lowerSpeed) { //to run both belts at the same time 
        runUpperBelt(upperSpeed);
        runLowerBelt(lowerSpeed);
    }

    public void stopTower() {
        runTowerBelts(0.0, 0.0);
    }

    public boolean sensesBallBottom(){
        return (bottomSensor2.get() && bottomSensor3.get());
    }

    public boolean sensesBallMiddle(){
        return middleSensor1.get();
    } 

    public boolean sensesBallTop(){
        return topSensor0.get();
    } 

    /* Analog input stuff (just in case)
    public boolean sensesBallBottom(){
        return (bottomSensor2.getAverageVoltage() < 2.0);
    }

    public boolean sensesBallMiddle(){
        return (middleSensor1.getAverageVoltage() < 2.0);
    } 

    public boolean sensesBallTop(){
        return (topSensor0.getAverageVoltage() < 2.0);
    } 

    public double bottomSensorVoltage(){
        return bottomSensor2.getVoltage();
    }

    public double middleSensorVoltage(){
        return middleSensor1.getVoltage();
    } 

    public double topSensorVoltage(){
        return topSensor0.getVoltage();
    } 
    */
}