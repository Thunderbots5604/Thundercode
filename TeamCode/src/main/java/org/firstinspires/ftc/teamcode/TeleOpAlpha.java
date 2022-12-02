package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotparts.DriveTrain;

@TeleOp(name = "Alpha", group = "competition")
public class TeleOpAlpha extends OpMode {
    DriveTrain drive;

    @Override
    public void start() {
        drive = new DriveTrain(hardwareMap, "fl", "fr", "bl", "br");
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        drive.setPowersToZero();
        drive.calculatePower(new double[] {gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x});
        drive.normalizePowers();
        drive.pushPowers();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}