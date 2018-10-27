package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name="intake test", group="test")
public class intakeTest extends OpMode {
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;

    private Servo leftConveyor;
    private Servo rightConveyor;
    private Servo dumper;

    public void init() {
        leftIntake = hardwareMap.dcMotor.get("LeftIntake");
        rightIntake = hardwareMap.dcMotor.get("RightIntake");

        leftConveyor = hardwareMap.servo.get("leftConveyor");
        rightConveyor = hardwareMap.servo.get("rightConveyor");

        dumper = hardwareMap.servo.get("dumper");
        //servo stuff below

        leftConveyor.setPosition(0.5);
        rightConveyor.setPosition(0.5);
        dumper.setPosition(1);

        //motor stuff below
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void start() {
    }

    public void loop() {
        if (gamepad1.right_bumper) {
            leftConveyor.setPosition(1);
            rightConveyor.setPosition(0);
        } else if(gamepad1.right_trigger > .2) {
            leftConveyor.setPosition(0);
            rightConveyor.setPosition(1);
        } else {
            leftConveyor.setPosition(0.5);
            rightConveyor.setPosition(0.5);
        }

        leftIntake.setPower(-0.75*gamepad1.left_stick_y);
        rightIntake.setPower(-0.75*gamepad1.right_stick_y);

        dumper.setPosition(1-gamepad1.left_trigger);

        telemetry.addData("leftIntake", -0.75*gamepad1.left_stick_y);
        telemetry.addData("rightIntake", 0.75*gamepad1.right_stick_y);

        telemetry.update();
    }

    public void stop() {
        rightIntake.setPower(0);
        rightIntake.setPower(0);

        leftConveyor.setPosition(0.5);
        rightConveyor.setPosition(0.5);
    }



}


