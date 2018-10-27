package org.firstinspires.ftc.teamcode;
//This first line of code goes in every op mode written. It does something. probably.

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

//All these imports basically add themselves as you code. You can see each import kind of has a different purpose, and they'll be there depending on what kind of functions you have in your code.
//if the code is throwing an error saying "cannot resolve symbol", its probably because you are missing one of these. Clicking on the troubled code and pressing alt enter should insert the import to fix it.




//The purpose of this op mode is to create an organized and descriptive "repository" of all (or most...) of the code we used in in teleop last year.
//It will include explanations of movement functions (including encoder-based movement vs PID movement) and some other functionality we used last year, like vuforia and using color sensors.
//Don't forget to also refer to the sample op modes provided in the FTC app in android studio. They can be very helpful..





@Autonomous(name="BareBonesAuto", group="Mecanum Drive")
//This is just a naming scheme. You can @Autonomous if you want the code to show up in the auto section on the driver station, or @teleop if its teleop code.

@Disabled
//this @Disabled line is just remove op modes you aren't using from the Driver Station phone. I would @disable any programs you don't use anymore just to keep the Driver Station tidy.  Doing this can also prevent starting wrong or old programs by mistake.
public class BareBonesAuto extends LinearOpMode {
//LinearOpModes are generally used in autonomous (they have a "linear" line of thought, and Teleop opmodes loop over and over again.


    private ElapsedTime runtime = new ElapsedTime();
    //ElapsedTime variable allows you to display the elapsed runtime in telemetry, the little dialogue box on the driver station.
    //You can get the driver station to display all sorts of useful and fun info in the telemetry box, from motor power to current position to random messages like "Miles is a hippo"
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftIntake, rightIntake;

    private Servo leftConveyor;
    private Servo rightConveyor;
    private Servo leftFlipper;
    private Servo rightFlipper;
    private Servo relicArm1;
    private Servo gripperArm;
    private Servo jewelArm;

    private DcMotor rightLift;
    private DcMotor relicExtend;

    private ColorSensor colorSense;
    private DetectedColor teamColor = DetectedColor.BLUE;
    private PID pid;
    private BNO055IMU hero;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    ElapsedTime timer;
    //above we just declared all the op mode members. These are motors, servos, sensors, and any other things used in vision programs (like Vuforia).
    //Just pick names that are memorable and preferably the same names used in the hardware configuration for the robot.


    @Override
    public void runOpMode() throws InterruptedException {

      //Here are the actually instructions for this op mode. This is the heart of the Auto code and here you call on all of the differnet functions you can make. I reccomend a good turn function, move forward function, and functions for whatever else you may be doing in auto
        //Last year we had a function for figuring out the jewel color and a couple others.
        initRobot();
        waitForStart();

        vuforiaPictoScan();

        jewelArm.setPosition(1);
        Thread.sleep(1000);
        //These thread.sleeps are used to keep things running in order.
        DetectedColor scanColor = readJewel(1);
        if (scanColor != DetectedColor.UNSURE) {
            knockJewel(scanColor);
        }
        jewelArm.setPosition(0.4);
        Thread.sleep(1000);
        rotatePID(0);
    //I do NOT ENDORSE all these "magic numbers". You should definitely make constants for all of these important positions in order to keep the code neat and tidy!!!
        moveForward(0.75, 0.4);
        rotatePID(90);
        rotatePID(90);
        //last year our PID turn function was so shit that we had to call on it twice in a row to get the robot to the right position. Very slow, very bad, very not good.
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            moveForward(.15, 0.4);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            moveForward(.32, 0.4);
        } else {
            moveForward(.50, 0.4);
        }

        rotatePID(0);
        rotatePID(0);
        moveForward(.4, 0.4);

        timer.reset();
        while (opModeIsActive() && timer.time() <= .5) {
            rightIntake.setPower(-0.8);
            leftIntake.setPower(-0.8);
        }

        timer.reset();
        while (opModeIsActive() && timer.time() <= .5) {
            rightIntake.setPower(-0.8);
            leftIntake.setPower(-0.8);
            leftFrontMotor.setPower(-0.2);
            leftBackMotor.setPower(-0.2);
            rightFrontMotor.setPower(-0.2);
            rightBackMotor.setPower(-0.2);
            //If you watch one of our auto videos from last year, you can easily understand the sequence of events in this OpMode.
            //First scan pictothing with phone camera, then put jewel arm down, then figure out which jewel to knock, then do a little turn to knock jewel (an "if" statement in the "knockJewel" function)
            //PS, we also had an "unsure" option for the jewel, where we wouldn't knock either one bc out color sensor was shit
            //after jewel, we bring arm ip and move forward a certain amount, depending on the vuMark, we turn, move towards cryptocubby, dispense glyph (right intake and left intake spits out glyph)
            //then we go back and forth a little just to bump glyph in, and that's it.  Hopefully this year's autonomous will be a little smarter and better than last years. The builders won't admit it, but AUTO WINS CHAMPIONSHIPS. Coders jobs are the most important on the team.
        }
    }

    public void initRobot() {

        //
        // SET UP MOTORS AND SERVOS
        //

        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        rightLift = hardwareMap.dcMotor.get("rightLift");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        relicExtend = hardwareMap.dcMotor.get("relicExtend");

        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");

        leftConveyor = hardwareMap.servo.get("leftConveyor");
        rightConveyor = hardwareMap.servo.get("rightConveyor");

        relicArm1 = hardwareMap.servo.get("relicArm1");
        gripperArm = hardwareMap.servo.get("gripperArm");

        jewelArm = hardwareMap.servo.get("jewelArm");

        jewelArm.setPosition(.4);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        leftConveyor.setPosition(.5);
        rightConveyor.setPosition(.5);

        leftFlipper.setPosition(1);
        rightFlipper.setPosition(0);

        relicExtend.setPower(0);
        relicExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //
        // SET UP SENSORS
        //

        //  timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);  <-- This is another way you can add a timer to telemetry I think

        colorSense = hardwareMap.colorSensor.get("ColorSensor");
        colorSense.enableLed(false);
        //Here, we get the color sensor (named colorSense here) and can choose to turn the LED it has on or off.

        // sets up the gyro sensor
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hero = hardwareMap.get(BNO055IMU.class, "hero");
        hero.initialize(gyro_parameters);
        hero.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = "AfOI8sP/////AAAAmZg8zg4w00VjnDU010tFZWcX9+w/Y4KcH9Dc9U0rL1fOQuVYEzY4ah5vGZwEINRncG7CfD5fXsTrI0IQgsL1Jv2B53RL8rI34G9IBRXplRY8+k3zvyL8TRv5hjIcFCaRgA5Li5qKHRe3aNhpWMWIfqe/lzue6+iDg6vXKQZgKWP/e2t8gNZnoZzNLSkTI39a08uwY9pl9xsNvIA6zZX/P+ID3oSAFeh7K/347x2hd8e4QI18whJNAQ8Fpp/18vSBvJcM/VCN4XQPTsRQyVYjUM87gdpfYS4qWOw7slpq1m0zjgL9+hfuEZH3SreoTNQNqCmF8MTizap4Wpg636m/5VKSo1PhircgMH23647Bg44b";

        vuforia_parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }
    //this is the big chunk of code that are instructions for initializing the robot when you hit init on the driver station.
    //There are a couple of key jobs of this method. Get all of the motors, servos, and sensors from the hardware map on the driver station (to 'get' must correspond to the names assigned during the robot configuration),
    // setting up the gyro sensor with that big chunk of code (copied from the FTC Bno055imu op mode)
    //You must also set powers for motors and positions for servos in the init function.  You can reverse motors to make stuff easier later on, for exmaple you could reverse a drive motor to make programming drive more straight forward.



    //Here are all of the different functions for our auto. These are the most important parts. Making a bunch of these will just make your code beautiful, simple, reliable, neat, and dank.
    //Cannot emphasize enough the power of a well-made and tuned turn & drive straight functions
    //I'll go over each function a little more in detail
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
        //this color sensor function uses the color sensor to return a value, either red, blue, or unsure, which will be use by the readJewel function.  
    }

    private DetectedColor readJewel(int buffer) throws InterruptedException {
        DetectedColor scanColor = DetectedColor.UNSURE;
        for(int tries = 0; scanColor == DetectedColor.UNSURE && tries <= 5; tries++) {
            scanColor = getColor(buffer);
            telemetry.addData("tries", tries);
            telemetry.addData("scanned color", scanColor);
            //this function throws some variables into telemetry to display text on the driver station. always a neat & useful practice.
            telemetry.update();
            Thread.sleep(500);
        }
        return scanColor;
    //this DetectedColor function uses the getColor function to try 5 times to figure out which color ball it is looking at. If it gets "unsure" 5 times, it'll return unsure, but as soon as it gets a red or blue reading, it returns that.

    }

    private void knockJewel(DetectedColor scanColor) throws InterruptedException{
        if(scanColor == teamColor) {
            rotateDegrees(15,.15);
            jewelArm.setPosition(0);
            rotateDegrees(345,.15);
        } else {
            rotateDegrees(345,.15);
            jewelArm.setPosition(0);
            rotateDegrees(15,.15);
       //this is the step for dealing with the jewel. It is a simple turning function.
            // If the jewel was one color, we would rotate one direction to knock the jewel, bring the arm up, and rotate back to center.
            //if it was the other color, we would simply rotate the other direction and knock the opposite jewel.
            //P.S., this function uses "rotateDegrees, which is encoder-based turning instead of PID turning.
        }
    }

    private void knockJewelPID(DetectedColor scanColor) throws InterruptedException {
        if (scanColor == teamColor) {
            rotatePID(15);
            jewelArm.setPosition(0.4);
            rotatePID(0);
        } else {
            rotatePID(15);
            jewelArm.setPosition(0.4);
            rotatePID(0);
            }
    //this function is almost identical to the KnockJewel function above, except in this one we were going to use PID.
        //PID is generally much much more accurate than encoder-based turning, but we ended up using encoder turns sometimes bc our PID was so poorly tuned that turns took forever.
    }

    private void moveForward(double tiles, double speed) throws InterruptedException {
        resetEncoders();

        int sign = 1;
        double scaling = 1425; // empirically determined
        int pos = (int)(tiles * scaling);

        if (pos < 0) {
            sign = -1;
        }

        while(opModeIsActive() && sign * (pos - leftFrontMotor.getCurrentPosition()) >= 20) {
            leftFrontMotor.setPower(speed * sign);
            leftBackMotor.setPower(speed * sign);
            rightFrontMotor.setPower(speed * sign);
            rightBackMotor.setPower(speed * sign);
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        sign *= -1;
        speed *= 0.5;
        //One problem we had last year is the motors would rush by the target position, not detect that they were at the position we wanted,
        //and then keep driving straight forever, trying to get to an encoder value it already passed.
        //So the second half of the function basically tells the robot to go backwards at half speed (speed*=.5, sign*= -1) if it passes the target value by more than 20 encoder ticks.
        //It's a simple concept elegantly coded, a great example of some really quality code :)))) ;) ;)
        while(opModeIsActive() && sign * (pos - leftFrontMotor.getCurrentPosition()) >= 20) {
            leftFrontMotor.setPower(speed * sign);
            leftBackMotor.setPower(speed * sign);
            rightFrontMotor.setPower(speed * sign);
            rightBackMotor.setPower(speed * sign);
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        Thread.sleep(500);
   //this is the driving-straight function. so we designed our moveForward function to have a distance (measured in tiles) and a speed option.
        //We have a "scaling" factor that you can see. This is because the motors measure distance by encoder ticks(v. small distance), but we wanted to measure our auto by the 2ftx2ft tiles on the field.
        //If you want to do something like this for your movement function, you'll have to do some testing to see what value factor you need to get whatever metric you want (just by trial& err)


        //P.S. :  You could make a mecanum-strafe function pretty easily, just by reversing some wheel signs
    }

    private void rotateDegrees(int degrees, double speed) throws InterruptedException {
        resetEncoders();

        double scaling = 13.5; //empirically determined

        if(degrees <= 180) {
            int turn = (int)(degrees * scaling);
            while(opModeIsActive() && Math.abs(leftFrontMotor.getCurrentPosition() - turn) >= 10) {
                leftFrontMotor.setPower(speed);
                leftBackMotor.setPower(speed);
                rightFrontMotor.setPower(-speed);
                rightBackMotor.setPower(-speed);
                telemetry.addData("Diff", Math.abs(leftFrontMotor.getCurrentPosition() - turn));
                telemetry.update();
            }
        } else {
            degrees = 360-degrees;
            int turn = (int)(degrees * scaling);
            while(opModeIsActive() && Math.abs(-leftFrontMotor.getCurrentPosition() - turn) >= 10) {
                leftFrontMotor.setPower(-speed);
                leftBackMotor.setPower(-speed);
                rightFrontMotor.setPower(speed);
                rightBackMotor.setPower(speed);
                telemetry.addData("Diff", Math.abs(- leftFrontMotor.getCurrentPosition() - turn));
                telemetry.update();

                }
                //this is our encoder-based turning fucntion. we havent used it since we developed PID turning, but it's still an important function to understand
            //(like if PID suddenly shits itself the night before competition, its good to have a reliable turning system ready to go.
            //basically this turns the robot one direction if we say less than 180 deg, but if enter more than 180 deg, like 345 degrees, itll just turn 360-345 (15 deg) in the opposite direction.
        }

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        Thread.sleep(500);
    }

    private void rotatePID(int degrees) throws InterruptedException {
        resetEncoders();

        pid = new PID(0.006, 0.003, 0, 35, 1);

        pid.setTarget(degrees);

        while (opModeIsActive() && !pid.isAtTarget()) {

            double update = pid.getOutput(getHeading());

            telemetry.addData("PID Update", update);
            telemetry.addData("Target", pid.target);
            telemetry.addData("Heading", pid.heading);
            telemetry.addData("Error", pid.prevError);
            telemetry.addData("iTerm", pid.iTerm);
            telemetry.addData("dTerm", pid.dTerm);
            telemetry.addData("check", pid.check);
            telemetry.update();
            //telematry displays all values ^^^
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

   //PID turning, my favorite and arguably one of the most important weapons in 14039's pockets. PID turning, when tuned right, is POWERFUL
        //You can see the kP, Ki, kD, iThresh, and TargetThresh values in the fucntion. these need to be tuned. Again, the values will be different for differnet robot weights, wheels, weight distributions, friciton... etc
        //For our coders: if you have extra time, Take the mecanum wheel chassis we made this alst summer and try to tune a PId turn function to it. It is good practice to get good at tuning PID.
        //Once you get familiar with which P, I, and D values do what, you'll be able to tune it faster and more accurately.
        //One of the hardest things before a competition is tuning PID, b/c often you only have a day or two between when the robot is finished and competition, so only a day or two to tune PID to the specific robot.
    }

    private double getHeading() {
        Orientation angles = hero.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX); //may need to change axis unit to work with vertical hubs -- depending on how u orient hubs, axis may have to be differnet.
        double d = -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        if (d < 0) {
            d += 360;
        }
        return d;
        //this function allows you to get the orientation angle from the IMU sensor in the rev hubs (same sensor that figures out angles in PID)
    }

    private void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //resets encoders, must be done in between encoder-based movemnets, like drive-straight. Just resets the counters in the motors.
    }

    private void vuforiaPictoScan() {
        relicTrackables.activate();
        vuMark = RelicRecoveryVuMark.UNKNOWN;

        timer.reset();
        while (opModeIsActive() && timer.time() <= 2 && vuMark == RelicRecoveryVuMark.UNKNOWN) {
            //This is important because it tells the phone how many seconds it should scan for vuforia before giving up.
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Pictogram", vuMark);
            telemetry.update();
        }

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("Picture Not Found", 0);
            telemetry.update();
            vuMark = RelicRecoveryVuMark.CENTER;
        } else {
            telemetry.addData("Pictogram", vuMark);
            telemetry.update();
      //this is the image recognition system for the picto-graph from last year. It uses Vuforia, a pretty understand-able tool provided by FTC.
            //the camera scans for the vuMark (vuforia makes image recognition relatively easy & undertstandable. Not sure if we will be using it in Rover Ruckus.
            //But anyways the vuforia stuff scans and returns left, right, center, or unknown. Here we told it to just put it in the middle collumn if it comes out as "unknown"
        }
    }































}


