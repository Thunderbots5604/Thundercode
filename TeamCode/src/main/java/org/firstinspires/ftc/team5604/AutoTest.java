package org.firstinspires.ftc.team5604;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrainGyro;
import org.firstinspires.ftc.team5604.robotparts.Gyro;
import org.firstinspires.ftc.team5604.robotparts.Claw;


@Autonomous(name="test")
public class AutoTest extends LinearOpMode {

    private AutoDriveTrainGyro drive;
    private Gyro gyro;
    private Claw claw;

    @Override
    public void runOpMode() {
        gyro = new Gyro(hardwareMap);
        drive = new AutoDriveTrainGyro(hardwareMap, "fl", "fr", "bl", "br", new double[] {Values.LATERAL_ERROR, Values.LONGITUDINAL_ERROR, 0.01}, Values.INCHES_PER_TICK_LATERAL, Values.INCHES_PER_TICK_LONGITUDINAL, Values.RADIANS_PER_TICK, gyro);
        claw = new Claw(hardwareMap, "claw", 0.85, 1);
        waitForStart();
        claw.close();

        
        sleep(10000);
    }
}
