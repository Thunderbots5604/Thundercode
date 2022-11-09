package org.firstinspires.ftc.team5604;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Armon {
    private DcMotorEx armMotor;
    private DcMotorEx armMotor2;

    public Armon(HardwareMap map, String armName, String arm2Name) {
        armMotor = map.get(DcMotorEx.class, armName);
        armMotor2 = map.get(DcMotorEx.class, arm2Name);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean moveToTarget(double targetPos) {
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double actualPosition = (armMotor.getCurrentPosition() + armMotor2.getCurrentPosition()) / 2;
        if (actualPosition < targetPos + 100 && actualPosition > targetPos - 100) {
            return true;
        } else if (actualPosition < targetPos) {
            armMotor.setVelocity(armMotor.getPower() + Values.power);
            armMotor2.setVelocity(armMotor.getPower() + Values.power);
            return false;
        } else if (actualPosition > targetPos){
            armMotor.setVelocity(armMotor.getPower() - Values.power);
            armMotor2.setVelocity(armMotor.getPower() - Values.power);
            return false;
        }
        return false;
    }
    //how to use :)
    //while(!moveToTarget){
    //  moveToTarget();
    //}
}
