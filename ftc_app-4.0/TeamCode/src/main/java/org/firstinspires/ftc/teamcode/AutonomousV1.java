package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Disabled
@Autonomous(name="AutoV1", group="Mecanum Drive")
public class AutonomousV1 extends LinearOpMode
{
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    private DcMotor leftLift, rightLift;
    private DcMotor leftIntake, rightIntake;

    private Servo leftConveyor;
    private Servo rightConveyor;
    private Servo dumper;
    private Servo jewelArm;
    boolean reverse = false;

    private ColorSensor colorSense;
    private DetectedColor teamColor = DetectedColor.BLUE;
    private PID pid;
    private BNO055IMU hero;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    @Override
    public void runOpMode() throws InterruptedException {

        // starts robot
        initRobot();
        waitForStart();

        rotatePID(90);

        telemetry.addData("Done", 0);
        telemetry.update();
        Thread.sleep(3000);

        rotatePID(45);
        /*

        // Handles Jewel
        jewelArm.setPosition(1);
        Thread.sleep(1000);
        DetectedColor scanColor = readJewel(10);
        if (scanColor != DetectedColor.UNSURE) knockJewel(scanColor);
        jewelArm.setPosition(0);
        Thread.sleep(1000);

        // Does Glyph Shifting
        moveForward(1.9, .15);
        rotateDegrees(270,.15);
        moveForward(.6,.3);
        leftIntake.setPower(-1);
        rightIntake.setPower(-1);
        Thread.sleep(2500);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        moveForward(-.5,.15);
        moveForward(.7,.15);
        moveForward(-.2,.15);

        */
    }

    // Initializes All Motors for Robot
    public void initRobot() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        //leftLift = hardwareMap.dcMotor.get("leftLift");
        //rightLift = hardwareMap.dcMotor.get("rightLift");

        //leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        //dumper = hardwareMap.servo.get("dumper");

        leftConveyor = hardwareMap.servo.get("leftConveyor");
        rightConveyor = hardwareMap.servo.get("rightConveyor");
        jewelArm = hardwareMap.servo.get("jewelArm");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        //leftLift.setPower(0);
        //rightLift.setPower(0);

        leftConveyor.setPosition(0.5);
        rightConveyor.setPosition(0.5);
        //jewelArm.setPosition(0);
        //dumper.setPosition(1);

        colorSense = hardwareMap.colorSensor.get("ColorSensor");
        colorSense.enableLed(false);

        // sets up the gyro sensor
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled      = true;
        gyro_parameters.loggingTag          = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hero = hardwareMap.get(BNO055IMU.class, "hero");
        hero.initialize(gyro_parameters);
        hero.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = "AfOI8sP/////AAAAmZg8zg4w00VjnDU010tFZWcX9+w/Y4KcH9Dc9U0rL1fOQuVYEzY4ah5vGZwEINRncG7CfD5fXsTrI0IQgsL1Jv2B53RL8rI34G9IBRXplRY8+k3zvyL8TRv5hjIcFCaRgA5Li5qKHRe3aNhpWMWIfqe/lzue6+iDg6vXKQZgKWP/e2t8gNZnoZzNLSkTI39a08uwY9pl9xsNvIA6zZX/P+ID3oSAFeh7K/347x2hd8e4QI18whJNAQ8Fpp/18vSBvJcM/VCN4XQPTsRQyVYjUM87gdpfYS4qWOw7slpq1m0zjgL9+hfuEZH3SreoTNQNqCmF8MTizap4Wpg636m/5VKSo1PhircgMH23647Bg44b";

        vuforia_parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    private DetectedColor getColor(int buffer) {
        DetectedColor ret;
        colorSense.enableLed(true);
        int blue = colorSense.blue();
        int red = colorSense.red();
        if(Math.abs(blue-red) > buffer) {
            if(blue > red) ret = DetectedColor.BLUE;
            else ret = DetectedColor.RED;
        }
        else {
            ret = DetectedColor.UNSURE;
        }
        colorSense.enableLed(false);
        return ret;
    }

    private DetectedColor readJewel(int buffer) throws InterruptedException {
        //deploy arm
        DetectedColor scanColor = DetectedColor.UNSURE;
        for(int tries = 0; scanColor == DetectedColor.UNSURE && tries <= 5; tries++) {
            scanColor = getColor(buffer);
            telemetry.addData("tries", tries);
            telemetry.addData("scanned color", scanColor);
            telemetry.update();
            Thread.sleep(500);
        }
        return scanColor;
    }

    private void knockJewel(DetectedColor scanColor) throws InterruptedException{
        if(scanColor == teamColor) {
            rotateDegrees(30,.15);
            jewelArm.setPosition(0);
            rotateDegrees(330,.15);
        } else {
            rotateDegrees(330,.15);
            jewelArm.setPosition(0);
            rotateDegrees(30,.15);
        }
    }

    private void moveForward(double tiles, double speed) throws InterruptedException {
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

    private void rotateDegrees(int degrees, double speed) throws InterruptedException {
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



    private void rotatePID(int degrees) throws InterruptedException {

        // 0.008, 0.005, 0, -1, 0.5 <-- consistently overshoots
        // 0.008, 0.005, 0, 20, 0.5 <-- undershoots with P, and then I starts to build up and pushes it past target threshold
        // 0.008, 0.005, 0, 60, 0.5 <-- so it gets really close but then it takes a long time to get to 0. choosing to turn up I now
        // 0.008, 0.008, 0, 45, 0.5 <-- again it gets really close and then shoots past thresh
        // 0.008, 0.005, 0, 75, 0.5 <-- nails 90 degree turn but the correction is really slow. bumnping up I term and lowering iThresh
        // 0.008, 0.008, 0, 60, 0.5 <-- overshoots, error correction is really fukin slow, bumping up I, lowering iThresh, and lowering target thresh
        // 0.008, 0.011, 0, 35, 1 <-- under the mark. bumping up iThresh
        // 0.008, 0.011, 0, 40, 1 <-- damn that was really good. I'm gonna add more Ki for faster improvement but i liked it
        // 0.008, 0.013, 0, 35, 1 <-- ok this is a solid value. im keeping this

        pid = new PID(0.008, 0.013, 0, 35, 1);

        pid.setTarget(degrees);

        while (opModeIsActive() && !pid.isAtTarget()) {
            telemetry.addData("Target Thresh", pid.targetThresh);
            telemetry.addData("Previous Error", pid.prevError);

            double update = pid.getOutput(getHeading());

            telemetry.addData("PID Update", update);
            telemetry.addData("PID error", pid.prevError);
            telemetry.addData("PID iterm", pid.iTerm);
            telemetry.addData("PID dterm", pid.dTerm);
            telemetry.update();

            leftFrontMotor.setPower(-update);
            leftBackMotor.setPower(-update);
            rightFrontMotor.setPower(update);
            rightBackMotor.setPower(update);

            Thread.sleep(10);
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

    private double getHeading() {
        Orientation angles = hero.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX); //may need to change axis unit to work with vertical hubs
        return -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void vuforiaPictoScan() {
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Pictogram", vuMark);
            telemetry.update();
        }
        telemetry.addData("Pictogram", vuMark);
        telemetry.update();
    }
}



