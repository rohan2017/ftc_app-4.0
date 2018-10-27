package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="TankDrive", group="TankDrive")
public class TankDrive extends OpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    double x1, x2, y1, y2;

    DcMotor rightgrabber;
    DcMotor leftgrabber;


    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

//        rightgrabber = hardwareMap.dcMotor.get("rightintake");
//        leftgrabber = hardwareMap.dcMotor.get("leftintake");
//        leftgrabber.setPower(0);
        rightgrabber.setPower(0);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void start() {
    }

    public void loop() {
        y2 = -gamepad1.left_stick_y;
        y1 = -gamepad1.right_stick_y;
        x1 = -gamepad1.right_stick_x;
        x2 = -gamepad1.left_stick_x;

//        rightgrabber = -gamepad1.right_trigger;
//        leftgrabber = -gamepad1.left_trigger;

        telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);

        leftFrontMotor.setPower(y2+x2);
        leftBackMotor.setPower(y2-x2);
        rightFrontMotor.setPower(y1-x1);
        rightBackMotor.setPower(y1+x1);

        telemetry.update();
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
//        rightgrabber.setPower(0);
//        rightgrabber.setPower(0);
    }



        }


