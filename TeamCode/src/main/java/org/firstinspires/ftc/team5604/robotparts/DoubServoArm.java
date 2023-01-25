package org.firstinspires.ftc.team5604.robotparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubServoArm {
  
  private Servo arm;
  private double pos1;
  private double pos2;
  private double pos3;
    
    public Arm(HardwareMap map, String name, double targetPos) {
        arm1 = map.get(Servo.class, name); 
        arm2 = map.get(Servo.class, name);      
    }
  
  public void goPos1 (double pos1) {
        arm1.setPosition(pos1);
        arm2.setPosition(pos1);

  }
  
  public void goPos2 (double pos2) {
        arm1.setPosition(pos2);
        arm2.setPosition(pos2);
  }
  
  public void goPos3 (double pos3) {
        arm1.setPosition(pos3);
        arm2.setPosition(pos3);
  }
  
}
  
  
 
  
  
  
  




}
