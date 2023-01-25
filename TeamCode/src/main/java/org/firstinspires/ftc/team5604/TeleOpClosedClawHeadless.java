package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrainGyro;
import org.firstinspires.ftc.team5604.robotparts.Armon2;
import org.firstinspires.ftc.team5604.robotparts.Claw;

import org.firstinspires.ftc.team5604.robotparts.Gyro;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name = "Closed Claw Headless", group = "outreach")
public class TeleOpClosedClawHeadless extends OpMode {
    AutoDriveTrainGyro drive;
    Armon2 arm;
    Claw claw;
    Gyro gyro;
    
    boolean doneInit = false;
    
    boolean pastBack = false;
    boolean currentBack;
    boolean pastA = false;
    boolean currentA;
    boolean pastB = false;
    boolean currentB;
    boolean halfSpeed;
    double multiplier;
    
    boolean pastTriggered = false;
    boolean currentRightTrigger = false;
    boolean currentLeftTrigger = false;
    double rightTriggerValue = 0;
    double leftTriggerValue = 0;

    @Override
    public void start() {
        while(!doneInit){}
        claw.close();
    }

    @Override
    public void init() {
        arm = new Armon2(hardwareMap, "am1", "am2", 10, 1150, 0.05);
        claw = new Claw(hardwareMap, "claw", 0.85, 0.95);
        gyro = new Gyro(hardwareMap);
        drive = new AutoDriveTrainGyro(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, 0.01}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK, gyro);
        doneInit = true;
    }

    @Override
    public void loop() {
        telemetry.addData("position", Double.toString(arm.getCurrentPosition() - arm.getModifier()));
        telemetry.update();
        
        claw.close();
        
        drive.updateCurrentLocation();
        
        currentB = gamepad1.b;
        if(currentB && !pastB) {
            halfSpeed = !halfSpeed;
        }
        pastB = currentB;

        multiplier = halfSpeed ? 0.375 : 0.75;
        
        drive.setPowersToZero();
        double[] directions = drive.rotateDirections(new double[] {gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x}, -drive.getCurrentLocation()[2]);
        drive.calculatePower(directions);
        drive.normalizePowers();
        drive.scalePowers(multiplier);
        drive.pushPowers();
        
        currentBack = gamepad1.back;
        if(currentBack && !pastBack) {
            //arm.setZeroPosition();
        }
        pastBack = currentBack;
        
        currentA = gamepad1.a;
        /*if(currentA && !pastA) {
            claw.toggle();
        }*/
        pastA = currentA;
        
        rightTriggerValue = gamepad1.right_trigger;
        leftTriggerValue = gamepad1.left_trigger;
        if(rightTriggerValue > 0.5) {
            currentRightTrigger = true;
        }
        else if(leftTriggerValue > 0.5) {
            currentLeftTrigger = true;
        }
        
        if(currentRightTrigger) {
            arm.move(600 * ((4.0 / 3) * (rightTriggerValue - 0.25)));
        }
        else if(currentLeftTrigger) {
            arm.move(600 * ((-4.0 / 3) * (leftTriggerValue - 0.25)));
        }
        
        if(!(currentLeftTrigger || currentRightTrigger)) {
            if(!pastTriggered) {
                arm.lock();
            }
            arm.moveToTarget();
        }
        
        pastTriggered = currentRightTrigger || currentLeftTrigger;
        currentRightTrigger = false;
        currentLeftTrigger = false;
    }

    @Override
    public void stop() {
        drive.stop();
        arm.stop();
    }
}
