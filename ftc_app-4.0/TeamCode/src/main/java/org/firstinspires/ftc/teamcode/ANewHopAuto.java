package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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

import static java.lang.Math.abs;

@Autonomous(name="A New Hope AUTO", group="Linear Opmode")
//@Disabled
public class ANewHopAuto extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private BNO055IMU hero;
    double m1, m2, m3, m4;
    private PID pid;
    double x1, x2, y1, y2, s1, s2, s3;



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

    private void moveForward(double tiles, double speed) throws InterruptedException {
        resetEncoders();

        int sign = 1;
        double scaling = 1425; // empirically determined
        int pos = (int) (tiles * scaling);

        if (pos < 0) {
            sign = -1;
        }

        while (opModeIsActive() && sign * (pos - leftFrontMotor.getCurrentPosition()) >= 20) {
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
        while (opModeIsActive() && sign * (pos - leftFrontMotor.getCurrentPosition()) >= 20) {
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


    private double getHeading() {
        Orientation angles = hero.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX); //may need to change axis unit to work with vertical hubs -- depending on how u orient hubs, axis may have to be differnet.
        double d = -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
       /*
        if (d < 0) {
            d += 360;
        }
        */
        return d;
        //this function allows you to get the orientation angle from the IMU sensor in the rev hubs (same sensor that figures out angles in PID)
    }
    private void rotatePID(int degrees) throws InterruptedException {
        telemetry.addData("IMU-Output", getHeading());
        telemetry.addData("TargetAngle", degrees);
        resetEncoders();

        double power=0.03;
        while (opModeIsActive() && degrees>abs(getHeading()))  {
            leftFrontMotor.setPower(power);
            leftBackMotor.setPower(power);
            rightFrontMotor.setPower(-power);
            rightBackMotor.setPower(-power);
            Thread.sleep(10);

            telemetry.addData("IMU-Output", getHeading());
            telemetry.addData("TargetAngle", degrees);
            telemetry.update();

        }


        /*
        pid = new PID(0.006, 0.003, 0, 35, 1);

        pid.setTarget(degrees);

        while (opModeIsActive() && !pid.isAtTarget()) {

            double update = pid.getOutput(getHeading());

            //telemetry.addData("PID Update", update);
            //telemetry.addData("Target", pid.target);
            telemetry.addData("Heading", pid.heading);
            //telemetry.addData("Error", pid.prevError);
            //telemetry.addData("iTerm", pid.iTerm);
            //telemetry.addData("dTerm", pid.dTerm);
            //telemetry.addData("check", pid.check);
            telemetry.update();

            //parth - reduce motor power
            update = update/20;




            //telematry displays all values ^^^
            leftFrontMotor.setPower(-update);
            leftBackMotor.setPower(-update);
            rightFrontMotor.setPower(update);
            rightBackMotor.setPower(update);

            Thread.sleep(10);


        }
        */

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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters gyro_parameters= new BNO055IMU.Parameters();

        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        hero = hardwareMap.get(BNO055IMU.class, "hero");
        hero.initialize(gyro_parameters);
        hero.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
       // runtime.reset();
        rotatePID(90);

    }

}
