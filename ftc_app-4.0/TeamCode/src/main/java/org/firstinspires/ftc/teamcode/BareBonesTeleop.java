
//The purpose of this op mode is to create an organized and descriptive "repository" of all (or most...) of the code we used in in teleop last year.


package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//bunch of imports to get all the stuffs working right (if something cannot "resolve", you're prob missing an import. just hit alt-enter after pressing the thing that isnt resolving


@TeleOp(name="BareBonesTeleop", group="MecanumDrive")
@Disabled
//^disabled line takee code on and off driver station, name names it in driver station, and @Teleop puts it in teleop section.
public class BareBonesTeleop extends OpMode {
   //notice teleop extends OpMode instaed of LinearOpMode. This is beecause autonomous is "linear" (does one line of code once),
    //but teleop is in a loop (runs teleop code loop many times per second, to create a seamless control experience)

    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    double intakeSpeed = .6;
    double slow = 1;
    //  private DcMotor leftLift, rightLift;
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
    // private CRServo relicRetract;

    boolean reverse = false;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    double x1, x2, y1, y2;
//Again, here we are declaring all the stuff we are going to use in teleop. Motors, servos, variables like "slow"
    //for mecanum drive, we declare x1, x2, y1, y2, because we will need these to deal with math to get meacnums running,
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

        // relicRetract = hardwareMap.crservo.get("relicRetract");

        relicArm1 = hardwareMap.servo.get("relicArm1");
        gripperArm = hardwareMap.servo.get("gripperArm");

        jewelArm = hardwareMap.servo.get("jewelArm");


        jewelArm.setPosition(.4);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        leftIntake.setPower(0);
        rightIntake.setPower(0);


        leftConveyor.setPosition(.5);
        rightConveyor.setPosition(.5);

        leftFlipper.setPosition(1);
        rightFlipper.setPosition(0);


        relicArm1.setPosition(2);
        gripperArm.setPosition(0.35);

        // relicRetract.setPower(0);
        relicExtend.setPower(0);
        relicExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // sensorArm.setPosition(1);
        //   rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //init function "attatches" code to robot configuration on driver station by getting devices (motors, servos, sensors) from the hardware map
        //Also set powers for motors and positions for motors in init, and reverse anything u want to.
        //You can also set motors to "ZeroPowerBehavior.BRAKE". This is if you want the motor not to coast.  Driving with braking and without braking are two
        //ENTIRELY differnet experiences. I encourage you to steal the extra chassis and try out driving with and without drive motor braking, just to get an understanding.

    }
    @Override
    public void start() {
  //this code will run one time once you hit "start".  You could start a telemetry timer here or maybe something else. but it can also just be left blank.
    }
@Override
//these "overrides" arent necessary, but the sample op modes from FTC had them. IDK what they are good for..
    public void loop() {
        //this is the main loop. Loops until you hit stop.
        //Last year, almost our entire teleop code was written with a bunch of clever "if" statements. I think this is a good way to do most of teleop.

        y2 = -gamepad1.left_stick_y;
        y1 = -gamepad1.right_stick_y;
        x1 = gamepad1.right_stick_x;
        x2 = gamepad1.left_stick_x;

        telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);
        //Here you see the x1,x2,y1,y2 used in mecanum.
        //   sensorArm.setPosition(0);


        if(gamepad1.right_trigger > 0.1){
            slow = 0.4;
        } else {
            slow = 1;
        }
        if (!gamepad1.right_bumper) {
            leftFrontMotor.setPower(slow * (y2 + x2));
            leftBackMotor.setPower(slow * (y2 - x2));
            rightFrontMotor.setPower(slow * (-y1 + x1));
            rightBackMotor.setPower(slow * (-y1 - x1));
        } else {
            leftFrontMotor.setPower(slow * (-y1 - x1));
            leftBackMotor.setPower(slow * (-y1 + x1));
            rightFrontMotor.setPower(slow * (y2 - x2));
            rightBackMotor.setPower(slow * (y2 + x2));
        }
       //this is the driving function. This is the final piece of code needed to run mecanum wheels. Here we have it set up to drive normally, but if we press gamepad1 right bumper,
        //it reverses the robot (basically gives the driver a "reverse" button. Some drivers like it, others dont.

        if (gamepad1.right_bumper){
            jewelArm.setPosition(.9);
        } else{
            jewelArm.setPosition(.4);
        }
//        Here was an if statemnt that allowed us to control the jewel arm, in-case we needed it for something random.
//
// if (gamepad2.right_bumper) {
//            leftConveyor.setPosition(0);
//            rightConveyor.setPosition(1);
//        } else if (gamepad2.right_trigger > .2) {
//            leftConveyor.setPosition(1);
//            rightConveyor.setPosition(0);
//        } else {
//            leftConveyor.setPosition(0.5);
//            rightConveyor.setPosition(0.5);
//        }


        if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) {
            leftFlipper.setPosition(.52);
            rightFlipper.setPosition(.48);
        } else if (gamepad2.a) {
            leftFlipper.setPosition(.87);
            rightFlipper.setPosition(.13);
        } else {
            leftFlipper.setPosition(1);
            rightFlipper.setPosition(0);
      //another if-statement controls the flipper. We had a resting position, a "primed" position, and a "flipped" position.
            //Again, these "magic numbers" are kinda a bad practice, better to declare a variable so you can keep track of things better.
        }



        if (gamepad2.dpad_up) {

            rightLift.setPower(1);
        } else if (gamepad2.dpad_down) {
            rightLift.setPower(-1);
        } else{
            rightLift.setPower(0);

    //if statemnet brings lift up & down.
        }




        if (gamepad2.left_bumper) {
            relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicExtend.setPower(-1);
            //  relicRetract.setPower(-0.35);
        } else if (gamepad2.right_bumper){
            relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicExtend.setPower(.4);
            //  relicRetract.setPower(.5);
        }else{
            relicExtend.setPower(0);
            //  relicRetract.setPower(0);
       //relic arm goes in & out.
        }

        if (gamepad2.dpad_right){
            gripperArm.setPosition(.54);
        }
        else{
            gripperArm.setPosition(.24);
        }

        if (gamepad2.x){
            relicArm1.setPosition(0.05);
        } else{
            relicArm1.setPosition(2);

       //two servos controlling relic claw. another series of if statements with ugly "magic numbers"
        }
        leftIntake.setPower(gamepad2.left_stick_y );
        rightIntake.setPower(gamepad2.right_stick_y );
        //intake wheels were controlled by secondary driver's joysticks.

        // dumper.setPosition(1-gamepad1.left_trigger);

        telemetry.addData("reversed", reverse);
        telemetry.addData("leftIntake", 1* gamepad2.left_stick_y);
        telemetry.addData("rightIntake",  1*gamepad2.right_stick_y);

        telemetry.update();
    }
    @Override
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

   //stop button just turns all the motors off and puts servos back to resting positions!! Easy!!
    }



}

