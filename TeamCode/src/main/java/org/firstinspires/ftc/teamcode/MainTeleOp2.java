package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Main TeleOp2")

public class MainTeleOp2 extends LinearOpMode {

    public DcMotor Lift = null;
    public DcMotor Intake = null;
    public Servo Roll = null;
    public DcMotor Gecko = null;

    private boolean isPressed = false;

    public ElapsedTime runTime = new ElapsedTime();
    public boolean buttonPressed = false;
    public double lastTick = 0;
    public double lastTime = 0;
    public double currentTick = 0;
    public double currentTime = 0;
    public double currentShooterPower = 0;
    public double targetShooterRPM = 0;

    @Override
    public void runOpMode() {
        double speed;

        Lift = hardwareMap.get(DcMotor.class, "L");
        Intake = hardwareMap.get(DcMotor.class, "I");
        Roll = hardwareMap.get(Servo.class, "R");
        Gecko = hardwareMap.get(DcMotor.class, "Gek");
        Gecko.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            double Power = gamepad2.right_stick_y;
            double Duck = gamepad2.left_stick_y;

            //Intake
            if (gamepad2.x) {
                Intake.setPower(1);
            }
            else if (gamepad2.y) {
                Intake.setPower(-1);
            }
            else  {
                Intake.setPower(0);
            }

            //GRABBER
            if (gamepad2.right_bumper) {
                Roll.setPosition(1.5);
            }
            else if (gamepad2.left_bumper) {
                Roll.setPosition(0);
            }


            //slow button
            if(gamepad1.left_trigger > .1) {
                speed = .5;
            } else {
                speed = 1;
            }

            while(gamepad1.left_trigger > .1){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y/2,
                                -gamepad1.left_stick_x/2,
                                -gamepad1.right_stick_x/2
                        )
                );
            }

            //Intake.setPower(Zoom*(speed*0.75));
            Lift.setPower(Power*speed*0.8);
            Gecko.setPower(Duck*speed*0.65);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("a", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
