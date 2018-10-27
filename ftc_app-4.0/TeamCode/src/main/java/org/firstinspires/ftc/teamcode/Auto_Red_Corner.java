package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name="Auto_Red_Corner", group="Mecanum Drive")
public class Auto_Red_Corner extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    private DcMotor leftLift, rightLift;
    private DcMotor leftIntake, rightIntake;

    private Servo leftConveyor;
    private Servo rightConveyor;
    private Servo dumper;
    private Servo sensorArm;
    boolean reverse = false;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    ColorSensor colorSense;
    DetectedColor teamColor = DetectedColor.BLUE;

    double x1, x2, y1, y2;

    @Override
    public void runOpMode() throws InterruptedException{
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        dumper = hardwareMap.servo.get("dumper");

        leftConveyor = hardwareMap.servo.get("leftConveyor");
        rightConveyor = hardwareMap.servo.get("rightConveyor");
        sensorArm = hardwareMap.servo.get("sensorArm");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE); //why is this necessary @Vitchka?
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE); //for some reason setting encoder position to same value produces a turn instead of going straight

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        leftLift.setPower(0);
        rightLift.setPower(0);

        leftConveyor.setPosition(0.5);
        rightConveyor.setPosition(0.5);
        sensorArm.setPosition(1);
        dumper.setPosition(1);

        colorSense = hardwareMap.colorSensor.get("ColorSensor");
        colorSense.enableLed(false);

        waitForStart();

        // START LINEAR OP MODE HERE

        // moveForward(3, 0.5);
        sensorArm.setPosition(1);
        Thread.sleep(1000);
        //deploy arm
        DetectedColor scanColor = DetectedColor.UNSURE;
        for(int tries = 0; scanColor == DetectedColor.UNSURE && tries <= 5; tries++) {
            scanColor = getColor(10);
            telemetry.addData("tries", tries);
            telemetry.addData("scanned color", scanColor);
            telemetry.update();
            Thread.sleep(500);
        }
        if (scanColor != DetectedColor.UNSURE) knockJewel(scanColor);
        sensorArm.setPosition(0);
        //after dealing with the jewels - this goes after both if statements
        moveForward(1.9, .15);
        rotateDegrees(270,.15);
        moveForward(.6,.3);
        leftIntake.setPower(-1);
        rightIntake.setPower(-1);
        Thread.sleep(2500);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        moveForward(-.5,.15);
        //  moveBackward(.5, .15);
        moveForward(.7,.15);
        moveForward(-.2,.15);
        //moveBackward(.2, .15);
    }

    public DetectedColor getColor(int buffer){
        DetectedColor color;

        colorSense.enableLed(true);

        int blue = colorSense.blue();
        int red = colorSense.red();
        if(Math.abs(blue-red) > buffer) {
            if(blue > red) color = DetectedColor.BLUE;
            else color = DetectedColor.RED;
        } else color = DetectedColor.UNSURE;

        colorSense.enableLed(false);

        return color;
    }

    public void knockJewel(DetectedColor scanColor) throws InterruptedException{
        if(scanColor == teamColor) {
            rotateDegrees(340,.15);
            sensorArm.setPosition(0);
            rotateDegrees(20,.15);
        } else {
            rotateDegrees(20,.15);
            sensorArm.setPosition(0);
            rotateDegrees(340,.15);
        }

        moveForward(1.9, .15);
        rotateDegrees(270,.15);
        moveForward(.6,.3);
        leftIntake.setPower(-1);
        rightIntake.setPower(-1);
        Thread.sleep(2500);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        //
        // moveBackward(.5, .15);
        moveForward(-.5,.15);
        moveForward(.7,.15);
        //   moveBackward(.2, .15);
        moveForward(-.2,.15);
    }

    public void moveForward(double tiles, double speed) throws InterruptedException {
        resetEncoders();

        double scaling = 1425; //empirically determined
        int pos = (int)(tiles * scaling);
        leftFrontMotor.setTargetPosition(pos);
        leftBackMotor.setTargetPosition(pos);
        rightFrontMotor.setTargetPosition(pos);
        rightBackMotor.setTargetPosition(pos);

        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        Thread.sleep(5000); //todo: I'll see what happens with 5000 - may be enough to pull offsimple autonomous
    }



    public void rotateDegrees(int degrees, double speed) throws InterruptedException {
        resetEncoders();

        double scaling = 13.5; //empirically determined

        if(degrees <= 180) {
            int turn = (int)(degrees * scaling);
            leftFrontMotor.setTargetPosition(turn);
            leftBackMotor.setTargetPosition(turn);
            rightFrontMotor.setTargetPosition(-turn);
            rightBackMotor.setTargetPosition(-turn);
        } else {
            degrees = 360-degrees;
            int turn = (int)(degrees * scaling);

            leftFrontMotor.setTargetPosition(-turn);
            leftBackMotor.setTargetPosition(-turn);
            rightFrontMotor.setTargetPosition(turn);
            rightBackMotor.setTargetPosition(turn);
        }
        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        Thread.sleep((int)(degrees*2000/180/speed));
    }

    public void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

