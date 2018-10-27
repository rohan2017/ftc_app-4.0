package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="14039 A New Hope", group="MecanumDrive")
public class ANewHope extends OpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    double m1, m2, m3, m4;

    double x1, x2, y1, y2, s1, s2, s3;
//s1 anmd s2w are slowing variables.
@Override
    public void init() {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");


        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    @Override
    public void init_loop() {
        telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);
        telemetry.addData("fl", m1);
        telemetry.addData("bl", m2);
        telemetry.addData("rf ", m3);
        telemetry.addData("rb", m4);


    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            s1 = .6;
        }   else {
                s1 = 1;

        }

        if (gamepad1.left_bumper) {
            s2 = .6 ;
        }   else {
            s2 = 1;

        }

        s3 = 1- .5*(gamepad1.right_trigger);


        y2 = -gamepad1.left_stick_y;
        y1 = -gamepad1.right_stick_y;
        x1 = gamepad1.right_stick_x;
        x2 = gamepad1.left_stick_x;


        m1 = (-y1 - x1)*s1*s2*s3;
        m2 = (-y1 + x1)*s1*s2*s3;
        m3 = (y2 - x2)*s1*s2*s3;
        m4 = ( y2 + x2)*s1*s2*s3;
        leftFrontMotor.setPower((-y1 - x1)*s1*s2*s3);
        leftBackMotor.setPower((-y1 + x1)*s1*s2*s3);
        rightFrontMotor.setPower((y2 - x2)*s1*s2*s3);
        rightBackMotor.setPower(( y2 + x2)*s1*s2*s3);

        telemetry.addData("y1", y1);
        telemetry.addData("y2", y2);
        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);
        telemetry.addData("fl", m1);
        telemetry.addData("bl", m2);
        telemetry.addData("rf ", m3);
        telemetry.addData("rb", m4);



        //   telemetry.addData("y1", y1);
      //  telemetry.addData("y2", y2);
      //  telemetry.addData("x1", x1);
      //  telemetry.addData("x2", x2);


    }
    @Override
    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);


    }
}


