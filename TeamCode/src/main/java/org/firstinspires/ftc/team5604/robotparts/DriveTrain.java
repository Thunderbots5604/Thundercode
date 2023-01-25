package org.firstinspires.ftc.team5604.robotparts;

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

    //store current motor ticks
    private int flTickZero;
    private int frTickZero;
    private int blTickZero;
    private int brTickZero;

    public DriveTrain(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor) {
        frontLeftMotor = map.get(DcMotor.class, flMotor);
        frontRightMotor = map.get(DcMotor.class, frMotor);
        backLeftMotor = map.get(DcMotor.class, blMotor);
        backRightMotor = map.get(DcMotor.class, brMotor);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set to break
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

//fr and bl go backwards, vice versa

    public void normalizePowers() {
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if(max > 1) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }
    }
    
    public void scalePowers(double scale) {
        frontLeftPower *= scale;
        frontRightPower *= scale;
        backLeftPower *= scale;
        backRightPower *= scale;
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

    //sets the current tick values as the new zero values for ticks
    public void setZeroTicks() {
        flTickZero = frontLeftMotor.getCurrentPosition();
        frTickZero = frontRightMotor.getCurrentPosition();
        blTickZero = backLeftMotor.getCurrentPosition();
        brTickZero = backRightMotor.getCurrentPosition();
    }

    public int[] getTickChange() {
        int[] change = new int[4];

        change[0] = frontLeftMotor.getCurrentPosition() - flTickZero;
        change[1] = frontRightMotor.getCurrentPosition() - frTickZero;
        change[2] = backLeftMotor.getCurrentPosition() - blTickZero;
        change[3] = backRightMotor.getCurrentPosition() - brTickZero;

        return change;
    }
}
