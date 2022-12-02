package org.firstinspires.ftc.team5604.autonomous.blueside;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team5604.autonomous.DetectSignalSleeve;
import org.firstinspires.ftc.team5604.robotparts.DriveTrain;
import org.firstinspires.ftc.team5604.robotparts.Claw;
import org.firstinspires.ftc.team5604.robotparts.Armon;

@Autonomous(name = "BlueNearCap", group = "Autonomous")
public class BlueNearCap extends LinearOpMode {
    private DetectSignalSleeve camera;
    private int location;
    private Armon arm;
    private Claw claw;
    private DriveTrain drive;
    private ElapsedTime timer;

    /*
    private RobotPosition[] positions = new RobotPosition[] {
        new RobotPosition(start),
        new RobotPosition(storage),
        new RobotPosition(pole)
    }

    private RobotPosition[] parkPositions = new RobotPosition[] {
        new RobotPosition(first),
        new RobotPosition(second),
        new RobotPosition(third),
    }
     */

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
                    /*
                    drive to positions[1]
                     */
                    drivePosition = DrivePosition.POSITION_TOSTORAGE;
                    break;
                case POSITION_TOSTORAGE:
                    /*
                    do an if check for if the robot has reached the storage
                     */
                    if (true) {
                        drivePosition = DrivePosition.POSITION_STORAGE;
                        clawState = ClawState.CLAW_STORAGE;
                    }
                    break;
                case POSITION_STORAGE:
                    drive.stop();
                    break;
                case POSITION_TOPOLE:
                    /*
                    drive to positions[2], check if reached
                     */
                    if (true) {
                        armState = ArmState.ARM_POLE;
                        drivePosition = DrivePosition.POSITION_POLE;
                    }
                    break;
                case POSITION_POLE:
                    drive.stop();
                    break;
                case POSITION_TOPARK:
                    switch (location) {
                        case 1:
                            /*
                            drive to parkPositions[0]
                             */
                            break;
                        case 2:
                            /*
                            drive to parkPositions[1]
                             */
                            break;
                        case 3:
                            /*
                            drive to parkPositions[2]
                             */
                            break;
                        default:
                            /*
                            cry about it
                             */
                            break;
                    }
                    drivePosition = DrivePosition.POSITION_PARK;
                    break;
                case POSITION_PARK:
                    /*
                    if statement if robot has reached park position
                     */
                    if (true) {
                        complete[0] = true;
                    }
                    break;
                default:
                    drivePosition = DrivePosition.POSITION_START;
                    break;
            }
            switch (armState) {
                case ARM_START:
                    /*
                    send instruction to raise arm to cone level
                     */
                    armState = ArmState.ARM_TOSTORAGE;
                    break;
                case ARM_TOSTORAGE:
                    armState = ArmState.ARM_STORAGE;
                    break;
                case ARM_STORAGE:
                    //wait
                    break;
                case ARM_TOPOLE:
                    /*
                    raise arm to pole level, do if check for when high enough
                     */
                    if (true) {
                        drivePosition = DrivePosition.POSITION_TOPOLE;
                    }
                    break;
                case ARM_POLE:
                    /*
                    lower arm slightly so that cone falls snugly
                     */
                    clawState = ClawState.CLAW_POLE;
                    armState = ArmState.ARM_TOPARK;
                    break;
                case ARM_TOPARK:
                    /*
                    return arm to starting compact position
                     */
                    armState = ArmState.ARM_PARK;
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
                    /*
                    open claw
                     */
                    clawState = ClawState.CLAW_TOSTORAGE;
                    break;
                case CLAW_TOSTORAGE:
                    //wait
                    break;
                case CLAW_STORAGE:
                    /*
                    close claw
                     */
                    armState = ArmState.ARM_TOPOLE;
                    clawState = ClawState.CLAW_TOPOLE;
                    break;
                case CLAW_TOPOLE:
                    //wait
                    break;
                case CLAW_POLE:
                    /*
                    open claw
                     */
                    clawState = ClawState.CLAW_TOPARK;
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
            if (complete.equals(new boolean[] {true, true, true})) {
                stop();
            }
        }
    }
}
