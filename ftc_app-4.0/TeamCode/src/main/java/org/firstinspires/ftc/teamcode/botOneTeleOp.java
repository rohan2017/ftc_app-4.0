package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="botOneTeleOP", group="Mecanum Drive")
public class botOneTeleOp extends OpMode {
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftGobbler = null;
    private DcMotor rightGobbler = null;
    private Servo TopperPusher = null;
    private Servo BottomerPusher = null;
    private Servo LeftUpperDowner = null;
    private Servo RightUpperDowner = null;


    double x1, x2, y1, y2;




    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        leftGobbler = hardwareMap.dcMotor.get("LeftIntake");
        rightGobbler = hardwareMap.dcMotor.get("RightIntake");

        TopperPusher = hardwareMap.servo.get("TopEjector");
        BottomerPusher = hardwareMap.servo.get("BottomEjector");

        LeftUpperDowner = hardwareMap.servo.get("LeftElevator");
        RightUpperDowner = hardwareMap.servo.get("RightElevator");
//servo stuff below
        TopperPusher.setPosition(0);
        BottomerPusher.setPosition(1);

        LeftUpperDowner.setPosition(.5);
        RightUpperDowner.setPosition(.5);


        //motor stuff below
        leftGobbler.setPower(0);
        rightGobbler.setPower(0);

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
        if (gamepad1.a) {
            TopperPusher.setPosition(1);
            BottomerPusher.setPosition(0);
        }
        else {
            TopperPusher.setPosition(0);
            BottomerPusher.setPosition(1);
        }


        if (gamepad1.right_bumper) {
            LeftUpperDowner.setPosition(1);
            RightUpperDowner.setPosition(0);
        }
        else if  (gamepad1.left_bumper) {
            LeftUpperDowner.setPosition(0);
            RightUpperDowner.setPosition(1);
        }
        else{
            LeftUpperDowner.setPosition(.5);
            RightUpperDowner.setPosition(.5);
        }


        y2 = -gamepad1.left_stick_y;
        y1 = -gamepad1.right_stick_y;
        x1 = -gamepad1.right_stick_x;
        x2 = -gamepad1.left_stick_x;



        telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);

        leftFrontMotor.setPower(y2+x2);
        leftBackMotor.setPower(y2-x2);
        rightFrontMotor.setPower(-y1+x1);
        rightBackMotor.setPower(-y1-x1);

        telemetry.update();

        float intake = gamepad1.right_trigger;
        leftGobbler.setPower(-intake);
        rightGobbler.setPower(intake);
        float outtake = gamepad1.left_trigger;
        leftGobbler.setPower(outtake);
        rightGobbler.setPower(-outtake);


    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightGobbler.setPower(0);
        rightGobbler.setPower(0);
    }



}


