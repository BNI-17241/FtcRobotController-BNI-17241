package org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.Controls.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Spark17241.pedroPathing.Constants;

import java.util.List;


@Autonomous(name = "Pedro Limelight og oz", group = "Drive")
public class MultiPointTest extends OpMode {

    public Follower follower;
    public Timer pathTimer, opmodeTimer;



//    protected final Pose PPGPose = new Pose(48, 81, Math.toRadians(180)); // closest to human 21
//    protected final Pose PPGPosePickup = new Pose(32, 81, Math.toRadians(180));
//
//    protected final Pose PGPPose = new Pose(48, 57.5, Math.toRadians(180)); // secount clostest 22
//    protected final Pose PGPPosePickup = new Pose(32, 57.5, Math.toRadians(180));
//
//    protected final Pose GPPPose = new Pose(48, 34, Math.toRadians(180)); // third closest 23
//    protected final Pose GPPPosePickup = new Pose(32, 34, Math.toRadians(180));


    protected PathChain Path1;
    protected PathChain Path2;
    protected PathChain Path3;
    protected PathChain Path4;
    protected PathChain Path5;
    public void pathGen() {
            Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(42.000, 8.500),
                                new Pose(100.000, 100.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 100.000),
                                    new Pose(42.000, 100.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 100.000),

                                    new Pose(100.000, 35.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 35.000),

                                    new Pose(43.000, 60.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.000, 60.000),

                                    new Pose(43.000, 12.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

    }


    @Override
    public void init() {
        pathGen();
    }

    @Override
    public void start(){}


    public enum pathingState {START, MOVE1, MOVE2, MOVE3, MOVE4, Move5, END}
    public pathingState pathState = pathingState.START;

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
        //telemetry.addData("Pose", Pose)

        // very simple movment test
        switch (pathState) {
            case START:
                follower.followPath(Path1);
                pathState = pathingState.MOVE1;
                break;
            case MOVE1:
                if (!(follower.isBusy())) {
                    follower.followPath(Path2);
                    pathState = pathingState.MOVE2;
                }
                break;
            case MOVE2:
                if (!(follower.isBusy())) {
                    follower.followPath(Path3);
                    pathState = pathingState.MOVE3;
                }
                break;
            case MOVE3:
                if (!(follower.isBusy())) {
                    follower.followPath(Path4);
                    pathState = pathingState.MOVE4;
                }
                break;
            case MOVE4:
                if (!(follower.isBusy())) {
                    follower.followPath(Path5);
                    pathState = pathingState.Move5;
                }
                break;
            case Move5:
                if (!(follower.isBusy())) {
                    pathState = pathingState.END;
                }
                break;
            case END:
                break;
        }
    }
}

//I added coment
