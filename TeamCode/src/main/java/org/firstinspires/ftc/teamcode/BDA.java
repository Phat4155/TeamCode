package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "BDA")
public class BDA extends LinearOpMode {

    Hardware4 robot = new Hardware4();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Sets Starting place on the field
        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //Gets robot to shoot power shots
        //Trajectory Duck1 = drive.trajectoryBuilder(startPose)
            //    .strafeLeft(5)
          //      .build();

        //Trajectory Duck2 = drive.trajectoryBuilder(Duck1.end())
          //      .forward(24.3)
            //    .build();

        //Gets robot across the line to drop the wobble goal
        //Trajectory Park = drive.trajectoryBuilder(Duck2.end())
            //    .splineToConstantHeading(new Vector2d(-73, 43), Math.toRadians(180))
          //      .build();

        //Trajectory Park2 = drive.trajectoryBuilder(Park.end())
          //      .forward(3)
            //    .build();

        //Waits for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        if (opModeIsActive()) {
            sleep(1000);
            robot.Forward(0.35);
            robot.Off();
            sleep(1000);
            robot.Up(0.6);
            robot.Off();
            sleep(3000);
            robot.Roll(0);
            sleep(1000);
            robot.Backward(0.37);
            robot.Off();
            robot.Roll(1.5);
            sleep(1000);
            robot.Down(0.3);
            robot.Off();
            sleep(1000);
            robot.Left(0.430);
            robot.Off();
            sleep(1000);
            robot.Forward(0.85);
            robot.Off();
        }
    }
}
