package org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation;

import org.firstinspires.ftc.teamcode.Biobuzz.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class Scheduler {

    public Queue<Task> tasks;

    private Task task;

    public Scheduler(){
        tasks = new LinkedList<>();
    }

    public Scheduler addTask(Task t){
        tasks.add(t);
        return this;
    }
    public boolean IsSchedulerDone(){
        return tasks.isEmpty();
    }
    public Scheduler waitForStill(){
        addTask(new Task() {
            @Override
            protected void Actions() {

            }
            @Override
            protected boolean Conditions() {
                return Localizer.getVelocity().x < 10 && Localizer.getVelocity().y < 10 && Localizer.getVelocity().h < Math.toRadians(3);
            }
        });
        return this;
    }

    public Scheduler waitSeconds(double sec){
        addTask(new Task() {
            private long wait;
            private long track;

            @Override
            protected void Actions() {
                wait = (long) (sec * 1000);
                track = -1;
            }

            @Override
            protected boolean Conditions() {
                if (track == -1) {
                    track = System.currentTimeMillis();
                    return false;
                }

                boolean r = (System.currentTimeMillis() - track) >= wait;
                if (r) track = -1;
                return r;
            }
        });
        return this;
    }

    public Scheduler StartPurePersuit(ArrayList<PurePersuit.Point>points, double TargetHeading, double radius){
        addTask(new Task() {
            @Override
            protected void Actions() {
                Chassis.PurePersuitTrajectory = new PurePersuit(points,radius);
                Chassis.usedTrajectory =Chassis.trajectoryStates.FOLLOWINGPUREPERSUIT;
            }

            @Override
            protected boolean Conditions() {
                return true;
            }
        });
        return this;
    }

    public Scheduler AddAnotherScheduler(Scheduler scheduler){
        tasks.addAll(scheduler.tasks);
        return this;
    }
    public void clear(){
        Task t = tasks.peek();
        tasks.clear();
    }

    public Scheduler DoingTaskForSeconds(double sec,Task task){//fa task continuu pentru un nr sec
        addTask(new Task() {
            private long wait;
            private long track;

            @Override
            protected void Actions() {
                wait = (long) (sec * 1000);
                track = -1;
                task.Actions();
            }

            @Override
            protected boolean Conditions() {
                if(!task.Conditions())
                {
                    track = -1;
                    return false;
                }
                if (track == -1) {
                    track = System.currentTimeMillis();
                    return false;
                }

                boolean r = (System.currentTimeMillis() - track) >= wait;
                if (r) track = -1;
                return r;
            }
        });
        return this;
    }

    public Scheduler DoingTaskForSecondsWithException(double sec,Task extraCondition,Task task){//fa task continuu pentru un nr sec
        addTask(new Task() {
            private long wait;
            private long track;

            @Override
            protected void Actions() {
                wait = (long) (sec * 1000);
                track = -1;
                task.Actions();
            }

            @Override
            protected boolean Conditions() {
                if(!task.Conditions())
                {
                    track = -1;
                    return false;
                }
                if (track == -1) {
                    track = System.currentTimeMillis();
                    return false;
                }

                boolean r = (System.currentTimeMillis() - track) >= wait;
                if (r) track = -1;
                return r || extraCondition.Conditions();
            }
        });
        return this;
    }

    public Scheduler DoingTaskUntilSeconds(double sec,Task task){//fa task dar treci peste dupa un nr de sec
        addTask(new Task() {
            private long wait;
            private long track;

            @Override
            protected void Actions() {
                wait = (long) (sec * 1000);
                track = -1;
                task.Actions();
            }

            @Override
            protected boolean Conditions() {
                if (track == -1) {
                    track = System.currentTimeMillis();
                    return false;
                }

                boolean r = (System.currentTimeMillis() - track) >= wait;
                if (r) track = -1;
                return r || task.Conditions();
            }
        });
        return this;
    }

    public void update() {
        if (IsSchedulerDone()) {
            task = null;
            return;
        }

        if (task == null) {
            task = tasks.peek();
            if (task == null) return;

            task.Actions();
            return;
        }

        if (task.Conditions()) {
            tasks.poll();
            task = null;
        }
    }

}
