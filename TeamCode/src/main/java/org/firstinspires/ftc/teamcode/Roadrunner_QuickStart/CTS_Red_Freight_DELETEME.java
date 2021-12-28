package org.firstinspires.ftc.teamcode.Roadrunner_QuickStart;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware4;
import org.firstinspires.ftc.teamcode.Roadrunner_QuickStart.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous (name = "CTS TESTING - DELETE", group = "Autonomous Main")
@Disabled
public class CTS_Red_Freight_DELETEME extends LinearOpMode {

    private Servo Arme = null;
    public DcMotor Lift = null;

    Hardware4 robot = new Hardware4();

    @Override
    public void runOpMode() {

        Lift = hardwareMap.get(DcMotor.class, "L");
        Arme = hardwareMap.get(Servo.class, "R");
        msStuckDetectInit = 10;

        float Object = 0;

        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Sets Starting place on the field
        Pose2d startPose = new Pose2d(10, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Middle
        Trajectory Place1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-2, -49.5), Math.toRadians(90))
                .build();

        Trajectory Duck1 = drive.trajectoryBuilder(Place1.end())

                .splineToConstantHeading(new Vector2d(-57, -48), Math.toRadians(180))
                .build();

        Trajectory Park1 = drive.trajectoryBuilder(Duck1.end())
                .splineToConstantHeading(new Vector2d(-62, -31), Math.toRadians(-90))
                .build();

        //Top
        Trajectory Place2 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(4, -48), Math.toRadians(90))
                .build();

        Trajectory Duck2 = drive.trajectoryBuilder(Place2.end())
                .splineToConstantHeading(new Vector2d(-57, -52), Math.toRadians(0))
                .build();

        Trajectory Park2 = drive.trajectoryBuilder(Duck2.end())
                .splineToConstantHeading(new Vector2d(-64, -34), Math.toRadians(-90))
                .build();

        //Bottom
        Trajectory Place3 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(15
                        , -20), Math.toRadians(90))
                .build();

        Trajectory Duck3 = drive.trajectoryBuilder(Place2.end())
                .splineToConstantHeading(new Vector2d(-57, -52), Math.toRadians(0))
                .build();

        Trajectory Park3 = drive.trajectoryBuilder(Duck2.end())
                .splineToConstantHeading(new Vector2d(-62, -40), Math.toRadians(-90))
                .build();

        telemetry.addLine("Press Play To Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //Duck is in the middle
            if (Object < 300 && Object > 1) {
                telemetry.addLine("Duck = Middle");
                telemetry.update();
                sleep(1000);
                robot.Down(0.345);
                robot.Off();
                sleep(1000);
                drive.followTrajectory(Place1);
                robot.Grab(1.5);
                //robot.Down(0.4);
                //robot.Off();
                sleep(1000);
                robot.Grab(0.35);
                sleep(1000);
                drive.followTrajectory(Duck1);
                robot.Off();
                sleep(1000);
                drive.turn(Math.toRadians(-180));
                robot.Forward(0.23);
                robot.Off();
                robot.setShooterPower(0.18);
                sleep(3000);
                robot.LeftStrafe(0.1);
                drive.followTrajectory(Park1);
                Arme.setPosition(1);
                break;
            }
            //Duck is on the right
            else if ( Object > 300) {
                telemetry.addLine("Duck = Right");
                telemetry.update();
                sleep(1000);
                drive.followTrajectory(Place2);
                robot.Grab(1.5);
                robot.Down(0.4);
                robot.Off();
                sleep(1000);
                robot.Grab(0.35);
                sleep(1000);
                robot.Off();
                drive.followTrajectory(Duck2);
                sleep(1000);
                drive.turn(Math.toRadians(-180));
                robot.Forward(0.15);
                robot.Off();
                robot.setShooterPower(0.18);
                sleep(3000);
                robot.LeftStrafe(0.1);
                drive.followTrajectory(Park2);
                break;
            }
            //Duck is on the left
            else {
                telemetry.addLine("Duck = Left");
                telemetry.update();
                sleep(1000);
                drive.followTrajectory(Place3);
                drive.turn(Math.toRadians(90));
                robot.Up(0.3);
                robot.Off();
                sleep(1000);
                robot.Grab(0);
                sleep(3000);
                robot.Grab(1.5);
                sleep(1000);
                robot.Down(0.2);
                robot.Off();
                drive.turn(Math.toRadians(-90));
                robot.Off();
                //drive.followTrajectory(Duck3);
                //drive.turn(Math.toRadians(-180));
                //robot.Forward(0.15);
                //robot.Off();
                //robot.setShooterPower(0.18);
                //sleep(3000);
                //robot.LeftStrafe(0.1);
                //drive.followTrajectory(Park3);
                //Arme.setPosition(1);
                break;
            }
        }
    }

}

