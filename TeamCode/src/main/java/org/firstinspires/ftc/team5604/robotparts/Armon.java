package org.firstinspires.ftc.team5604.robotparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Armon {
    private DcMotorEx armMotor;
    private DcMotorEx armMotor2;

    private double positionModifier;
    private double targetPosition;
    private double powerIncrement;

    private double actualPosition;
    private double powerDecrement;

    private double armMotorCurrentPower;
    private double armMotor2CurrentPower;

    public final double TARGET_INCREMENT;
    public final double MAX_POSITION;

    public Armon(HardwareMap map, String armName, String arm2Name, double positionIncrement, double maxPosition, double powerIncrement) {
        armMotor = map.get(DcMotorEx.class, armName);
        armMotor2 = map.get(DcMotorEx.class, arm2Name);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        TARGET_INCREMENT = positionIncrement;
        MAX_POSITION = maxPosition;
        this.powerIncrement = powerIncrement;

        positionModifier = (armMotor.getCurrentPosition() + armMotor2.getCurrentPosition()) / 2.0;
        targetPosition = getCurrentPosition();

        armMotorCurrentPower = 0;
        armMotor2CurrentPower = 0;
    }

    public boolean moveToTarget() {
        /*armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        actualPosition = getCurrentPosition();
        if (actualPosition < targetPosition + 10 && actualPosition > targetPosition - 10) {
            armMotor.setVelocity(1);
            armMotor2.setVelocity(1);
            return true;
        } else if (actualPosition < targetPosition) {
            armMotor.setVelocity(500);
            armMotor2.setVelocity(500);
            return false;
        } else {
            armMotor.setVelocity(0);
            armMotor2.setVelocity(0);
            return false;
        }
    }

    public double getCurrentPosition() {
        return ((armMotor.getCurrentPosition() + armMotor2.getCurrentPosition()) / 2.0) - positionModifier;
    }

    public void setZeroPosition() {
        positionModifier = armMotor.getCurrentPosition() + armMotor2.getCurrentPosition() / 2.0;
        targetPosition = getCurrentPosition();
    }

    public void incrementTargetPosition() {
        if(!(targetPosition > MAX_POSITION)) {
            targetPosition += TARGET_INCREMENT;
        }
    }

    public void decrementTargetPosition() {
        if(targetPosition + 50 > TARGET_INCREMENT) {
            targetPosition -= TARGET_INCREMENT;
        }
    }

    //how to use :)
    //Value set to hold
    public void hold()
    {
        armMotor.setPower(0.05);
        armMotor2.setPower(0.05);
    }

    //Stops
    public void stop(){
        armMotor.setPower(0);
        armMotor2.setPower(0);
    }

    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    public double getTargetPosition(double target) {
        return targetPosition;
    }

    public double getModifier() {
        return positionModifier;
    }
}
