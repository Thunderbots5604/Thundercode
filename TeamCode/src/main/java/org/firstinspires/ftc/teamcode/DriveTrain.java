import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
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

    public DriveTrain(HardwareMap map, String fLMotor, String fRMotor, String bLMotor, String bRMotor,
                      double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        frontLeftMotor = map.get(DcMotor.class, fLMotor);
        frontRightMotor = map.get(DcMotor.class, fRMotor);
        backLeftMotor = map.get(DcMotor.class, bLMotor);
        backRightMotor = map.get(DcMotor.class, bRMotor);

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

    public void calculatePower() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        frontLeftMotor.setPower(y + x + turn);
        backLeftMotor.setPower(y - x + turn);
        frontRightMotor.setPower(y - x - turn);
        backRightMotor.setPower(y + x - turn);
    }
}

