package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp(name="MARK4", group="MecanumDrive")
public class MARK4 extends OpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    //  private DcMotor leftLift, rightLift;
    private DcMotor leftIntake, rightIntake;

    private Servo leftConveyor;
    private Servo rightConveyor;
    private Servo leftFlipper;
    private Servo rightFlipper;
    private Servo relicArm1;
    private Servo relicArm2;

    private DcMotor rightLift;
    private DcMotor relicExtend;
    private CRServo relicRetract;

    boolean reverse = false;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    double x1, x2, y1, y2;

    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");


        rightLift = hardwareMap.dcMotor.get("rightLift");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        relicExtend = hardwareMap.dcMotor.get("relicExtend");

        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");

        leftConveyor = hardwareMap.servo.get("leftConveyor");
        rightConveyor = hardwareMap.servo.get("rightConveyor");

        relicRetract = hardwareMap.crservo.get("relicRetract");

        relicArm1 = hardwareMap.servo.get("relicArm1");
        relicArm2 = hardwareMap.servo.get("relicArm2");

        //   sensorArm = hardwareMap.servo.get("sensorArm");

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        leftIntake.setPower(0);
        rightIntake.setPower(0);


        leftConveyor.setPosition(0.5);
        rightConveyor.setPosition(0.5);

        leftFlipper.setPosition(.8);
        rightFlipper.setPosition(.2);


        relicArm1.setPosition(0);
        relicArm2.setPosition(0);

        relicRetract.setPower(0);
        relicExtend.setPower(0);
        relicExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // sensorArm.setPosition(1);
    }

    public void start() {
    }

    public void loop() {
        y2 = -gamepad1.left_stick_y;
        y1 = -gamepad1.right_stick_y;
        x1 = gamepad1.right_stick_x;
        x2 = gamepad1.left_stick_x;

        telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);

        //   sensorArm.setPosition(0);

        if (!gamepad1.right_bumper) {
            leftFrontMotor.setPower(y2 + x2);
            leftBackMotor.setPower(y2 - x2);
            rightFrontMotor.setPower(-y1 + x1);
            rightBackMotor.setPower(-y1 - x1);
        } else {
            leftFrontMotor.setPower(-y1 - x1);
            leftBackMotor.setPower(-y1 + x1);
            rightFrontMotor.setPower(y2 - x2);
            rightBackMotor.setPower( y2 + x2);
        }

        if (gamepad2.right_bumper) {
            leftConveyor.setPosition(0);
            rightConveyor.setPosition(1);
        } else if (gamepad2.right_trigger > .2) {
            leftConveyor.setPosition(1);
            rightConveyor.setPosition(0);
        } else {
            leftConveyor.setPosition(0.5);
            rightConveyor.setPosition(0.5);
        }




        if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) {
            leftFlipper.setPosition(.25);
            rightFlipper.setPosition(.75);
        } else if (gamepad2.a) {
            leftFlipper.setPosition(.7);
            rightFlipper.setPosition(.3);
        } else {
            leftFlipper.setPosition(.8);
            rightFlipper.setPosition(.2);
        }



        if (gamepad2.dpad_up) {

            rightLift.setPower(1);
        } else if (gamepad2.dpad_down) {
            rightLift.setPower(-1);
        } else{
            rightLift.setPower(0);
        }




        if (gamepad2.dpad_left) {
                relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                relicExtend.setPower(-1);
                relicRetract.setPower(0.75);
                } else if (gamepad2.dpad_right){
                relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                relicRetract.setPower(-.75);
                }else{
                relicExtend.setPower(0);
                relicRetract.setPower(0);
                }

                if (gamepad2.b){
                relicArm2.setPosition(.6);
                }
                else{
                relicArm2.setPosition(0);
                }

                if (gamepad2.x){
                relicArm1.setPosition(.6);
                } else{
                relicArm1.setPosition(0);
                }
                leftIntake.setPower(gamepad2.left_stick_y);
                rightIntake.setPower(gamepad2.right_stick_y);


                // dumper.setPosition(1-gamepad1.left_trigger);

                telemetry.addData("reversed", reverse);
                telemetry.addData("leftIntake", 0.75 * gamepad2.left_stick_y);
                telemetry.addData("rightIntake", 0.75 * gamepad2.right_stick_y);

                telemetry.update();
                }

public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        rightIntake.setPower(0);
        leftIntake.setPower(0);

        //   leftLift.setPower(0);
        //   rightLift.setPower(0);

        leftConveyor.setPosition(0.5);
        rightConveyor.setPosition(0.5);
        leftFlipper.setPosition(.9);
        rightFlipper.setPosition(.1);
        }



        }


