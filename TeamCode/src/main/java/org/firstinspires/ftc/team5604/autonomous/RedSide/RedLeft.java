package org.firstinspires.ftc.team5604.autonomous.RedSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;
import org.firstinspires.ftc.team5604.robotparts.AutoDriveTrain;
import org.firstinspires.ftc.team5604.robotparts.Claw;
import org.firstinspires.ftc.team5604.robotparts.Armon2;
import org.firstinspires.ftc.team5604.Values;

@Autonomous(name = "RedLeft", group = "Autonomous")
public class RedLeft extends LinearOpMode {
    private AutoDriveTrain driveTrain;
    private DetectSignalSleeve camera;
    private int location;
    private Armon2 arm;
    private Claw claw;
    private AutoDriveTrain drive;
    private ElapsedTime timer;

    private final double ARM_STARTING_HEIGHT = 100;
    private final double ARM_POLE_HEIGHT = 1100;
//    private final int NUM_CYCLES = 2;


    private double[][] positions = {{-Values.TILE_LENGTH, Values.TILE_LENGTH * 2.5, Math.PI/4},
                                        {0, Values.TILE_LENGTH * 3, -Math.PI/4}}; //[0]=storage, [1]=pole

    private double[][] parkPositions = {{-3100, 200, 0}, {0, 2700, 0}, {3100, 2700, 0}};


    enum DrivePosition {
        POSITION_START,
        POSITION_TOSTORAGE,
        POSITION_STORAGE,
        POSITION_TOPOLE,
        POSITION_POLE,
        POSITION_TOPARK,
        POSITION_PARK
    }

    enum ArmState {
        ARM_START,
        ARM_TOSTORAGE,
        ARM_STORAGE,
        ARM_TOPOLE,
        ARM_POLE,
        ARM_TOPARK,
        ARM_PARK
    }

    enum ClawState {
        CLAW_START,
        CLAW_TOSTORAGE,
        CLAW_STORAGE,
        CLAW_TOPOLE,
        CLAW_POLE,
        CLAW_TOPARK,
        CLAW_PARK
    }

    volatile DrivePosition drivePosition = DrivePosition.POSITION_START;
    volatile ArmState armState = ArmState.ARM_START;
    volatile ClawState clawState = ClawState.CLAW_START;

    private boolean[] complete = {false, false, false};

    @Override
    public void runOpMode() throws InterruptedException {
        //init stuff here
        camera = new DetectSignalSleeve(hardwareMap, telemetry, this);
        camera.init();
        waitForStart();
        location = camera.findRegion();
        sleep(500);
        timer = new ElapsedTime();

        while (opModeIsActive()) {
            switch (drivePosition) {
                case POSITION_START:
                    drive.setTargetLocation(new double[] {0, 200, 0}); //so robot doesn't scratch wall
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        drivePosition = DrivePosition.POSITION_TOSTORAGE;
                    }
                    break;
                case POSITION_TOSTORAGE:
                    drive.setTargetLocation(positions[0]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        drivePosition = DrivePosition.POSITION_STORAGE;
                    }
                    break;
                case POSITION_STORAGE:
                    clawState = ClawState.CLAW_STORAGE;
                    drive.stop();
                    break;
                case POSITION_TOPOLE:
                    drive.setTargetLocation(positions[1]);
                    if(!drive.moveToTargetLocation(.5)) {
                        drive.updateCurrentLocation();
                    } else {
                        armState = ArmState.ARM_POLE;
                        drivePosition = DrivePosition.POSITION_POLE;
                    }
                    break;
                case POSITION_POLE:
                    drive.stop();
                    break;
                case POSITION_TOPARK:
                    drive.setTargetLocation(parkPositions[location-1]);
                    if(!drive.moveToTargetLocation(.5)){
                        drive.updateCurrentLocation();
                    } else {
                        drivePosition = DrivePosition.POSITION_PARK;
                    }
                    break;
                case POSITION_PARK:
                    complete[0] = true;
                    break;
                default:
                    drivePosition = DrivePosition.POSITION_START;
                    break;
            }
            switch (armState) {
                case ARM_START:
                    armState = ArmState.ARM_TOSTORAGE;
                    break;
                case ARM_TOSTORAGE:
                    arm.setTargetPosition(ARM_STARTING_HEIGHT);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_STORAGE;
                    }
                    break;
                case ARM_STORAGE:
                    arm.lock();
                    arm.moveToTarget();
                    break;
                case ARM_TOPOLE:
                    arm.setTargetPosition(ARM_POLE_HEIGHT);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_POLE;
                    }
                    break;
                case ARM_POLE:
                    clawState = ClawState.CLAW_POLE;
                    arm.lock();
                    arm.moveToTarget();
                    break;
                case ARM_TOPARK:
                    arm.setTargetPosition(0);
                    if(arm.moveToTarget()){
                        armState = ArmState.ARM_PARK;
                    }
                    break;
                case ARM_PARK:
                    complete[1] = true;
                    break;
                default:
                    armState = ArmState.ARM_START;
                    break;
            }
            switch (clawState) {
                case CLAW_START:
                    claw.open();
                    clawState = ClawState.CLAW_TOSTORAGE;
                    break;
                case CLAW_TOSTORAGE:
                    //wait
                    break;
                case CLAW_STORAGE:
                    //closes claw only when both the arm is at the correct height and the robot is at
                    //the right location (since POSITION_STORAGE leads to CLAW_STORAGE, we can assume
                    //that the robot is already at the correct location; this if statement is only
                    //to check if the arm is at the storage)
                    if(armState == ArmState.ARM_STORAGE){
                        claw.close();
                        armState = ArmState.ARM_TOPOLE;
                        clawState = ClawState.CLAW_TOPOLE;
                        drivePosition = DrivePosition.POSITION_TOPOLE;
                    }
                    break;
                case CLAW_TOPOLE:
                    //wait
                    break;
                case CLAW_POLE:
                    //opens claw only when both the arm is at the correct height and the robot is at
                    //the pole (since ARM_POLE leads to CLAW_POLE, we can assume that the arm is already
                    //at the correct height; this if statement is only to check if the robot itself
                    //is at POSITION_POLE)
                    if(drivePosition == DrivePosition.POSITION_POLE) {
                        claw.open();
                        clawState = ClawState.CLAW_TOPARK;
                        armState = ArmState.ARM_TOPARK;
                        drivePosition = DrivePosition.POSITION_TOPARK;
                    }
                    break;
                case CLAW_TOPARK:
                    clawState = ClawState.CLAW_PARK;
                    break;
                case CLAW_PARK:
                    complete[2] = true;
                    break;
                default:
                    clawState = ClawState.CLAW_START;
                    break;
            }
            telemetry.update();
            if (complete[0] && complete[1] && complete[2]) {
                stop();
            }
        }
    }
}
