package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team5604.robotparts.Armon;

@Autonomous(name = "Armon", group = "competition")

public class ArmonTest extends LinearOpMode {

@Override
    public void runOpMode(){
        
        waitForStart();
        
        Armon arm_on = new Armon(hardwareMap, "am1", "am2", 1, 1000, 0.005);
        
        arm_on.setTargetPosition(1000);
        arm_on.moveToTarget();
    }

   

}