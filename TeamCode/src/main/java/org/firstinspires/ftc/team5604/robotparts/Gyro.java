package org.firstinspires.ftc.team5604.robotparts;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Gyro", group = "")
@Disabled
public class Gyro extends LinearOpMode {

    public BNO055IMU imu;

    @Override
    public void runOpMode() {}
    public Gyro(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        //hang until gyro is calibrated
        while(!imu.isGyroCalibrated()) {}

    }
    public double formatAngle(AngleUnit angleUnit, double angle) {
        return AngleUnit.RADIANS.fromUnit(angleUnit, angle);
    }
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}