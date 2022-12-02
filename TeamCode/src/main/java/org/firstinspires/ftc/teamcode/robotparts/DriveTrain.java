package org.firstinspires.ftc.teamcode.robotparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public DriveTrain(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor){
        frontLeftMotor = map.get(DcMotor.class, flMotor);
        frontRightMotor = map.get(DcMotor.class, frMotor);
        backLeftMotor = map.get(DcMotor.class, blMotor);
        backRightMotor = map.get(DcMotor.class, brMotor);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        this.frontLeftPower = 0;
        this.frontRightPower = 0;
        this.backLeftPower = 0;
        this.backRightPower = 0;
    }

    public void calculatePower(double[] inputs) {
        double y = inputs[1];
        double x = inputs[0];
        double turn = inputs[2];

        frontLeftPower = y + x + turn;
        backLeftPower = y - x + turn;
        frontRightPower = y - x - turn;
        backRightPower = y + x - turn;
    }

    public void normalizePowers() {
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if(max > 1) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }
    }

    public void setPowersToZero() {
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
    }

    public void pushPowers() {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}

