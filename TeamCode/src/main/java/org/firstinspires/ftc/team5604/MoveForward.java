package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team5604.robotparts.DriveTrain;

@Autonomous

public class MoveForward extends LinearOpMode{
    DriveTrain drive;    
    
    @Override
    public void runOpMode() {
        drive = new DriveTrain(hardwareMap, "fl", "fr", "bl", "br");
        waitForStart();
        
        drive.setPowersToZero();
        drive.calculatePower(new double[] {0, .75, 0});
        drive.normalizePowers();
        drive.pushPowers();
        
        sleep(500);
        
        drive.stop();
    }
}
    