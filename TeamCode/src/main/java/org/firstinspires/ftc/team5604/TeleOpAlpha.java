package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team5604.robotparts.DriveTrain;
import org.firstinspires.ftc.team5604.robotparts.Armon;
import org.firstinspires.ftc.team5604.robotparts.Claw;

@TeleOp(name = "Alpha", group = "competition")
public class TeleOpAlpha extends OpMode {
    DriveTrain drive;
    Armon arm;
    Claw claw;

    boolean doneInit = false;

    boolean pastBack = false;
    boolean currentBack;
    boolean pastA = false;
    boolean currentA;
    boolean pastB = false;
    boolean currentB;
    boolean halfSpeed;
    double multiplier;

    @Override
    public void start() {
        while(!doneInit){}
        claw.close();
    }

    @Override
    public void init() {
        drive = new DriveTrain(hardwareMap, "fl", "fr", "bl", "br");
        arm = new Armon(hardwareMap, "am1", "am2", 10, 1200, 0.05);
        claw = new Claw(hardwareMap, "claw", 0.85, 1);
        doneInit = true;
    }

    @Override
    public void loop() {
        telemetry.addData("position", Double.toString(arm.getCurrentPosition() - arm.getModifier()));
        telemetry.update();

        currentB = gamepad1.b;
        if(currentB && !pastB) {
            halfSpeed = !halfSpeed;
        }
        pastB = currentB;

        multiplier = halfSpeed ? 0.5 : 1;

        drive.setPowersToZero();
        drive.calculatePower(new double[] {multiplier * gamepad1.left_stick_x, -multiplier * gamepad1.left_stick_y, multiplier * gamepad1.right_stick_x});
        drive.normalizePowers();
        drive.pushPowers();

        currentBack = gamepad1.back;
        if(currentBack && !pastBack) {
            arm.setZeroPosition();
        }
        pastBack = currentBack;

        currentA = gamepad1.a;
        if(currentA && !pastA) {
            claw.toggle();
        }
        pastA = currentA;

        if(gamepad1.right_trigger > 0.5) {
            arm.incrementTargetPosition();
        }
        else if(gamepad1.left_trigger > 0.5) {
            arm.decrementTargetPosition();
        }

        arm.moveToTarget();
    }

    @Override
    public void stop() {
        drive.stop();
        arm.stop();
    }
}