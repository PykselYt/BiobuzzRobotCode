package org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;

public class AsyncScheduler{

    public ArrayList<Queue<Task>> asyncQueues;

    public AsyncScheduler() {
        super();
        asyncQueues = new ArrayList<>();
        asyncQueues.add(new LinkedList<>());   // index 0 = main queue
    }

    // Add async scheduler branch
    public AsyncScheduler AddAnotherAsyncScheduler(AsyncScheduler scheduler) {
        for(int i = 0;i<scheduler.asyncQueues.size();i++)
            this.asyncQueues.get(i).addAll(scheduler.asyncQueues.get(i));
        return this;
    }


    public AsyncScheduler addTask(Task t) {
        // ALWAYS add to asyncQueues[0], which is super.tasks
        asyncQueues.get(0).add(t);
        return this;
    }

    public AsyncScheduler waitSeconds(double sec){
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

    public AsyncScheduler DoingTaskForSeconds(double sec,Task task){//fa task continuu pentru un nr sec
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
    public AsyncScheduler DoingTaskForSecondsWithException(double sec,Task extraCondition,Task task){//fa task continuu pentru un nr sec
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
    public AsyncScheduler DoingTaskUntilSeconds(double sec,Task task){//fa task dar treci peste dupa un nr de sec
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

    // Wait for all async branches to finish
    public AsyncScheduler waitUntilAllAsyncDone() {
        addTask(new Task() {
            @Override protected void Actions() {}
            @Override protected boolean Conditions() {
                return asyncQueues.size() == 1;
            }
        });
        return this;
    }

    public boolean IsSchedulerSync(){
        return asyncQueues.size() == 1;
    }

    public void AddTasksToBeDoneAsync(AsyncScheduler scheduler){
        asyncQueues.add(scheduler.asyncQueues.get(0));
    }

    public void clearAsyncQueues() {
        // Clear only async queues (1+)
        for (int i = 1; i < asyncQueues.size(); i++) {
            Queue<Task> q = asyncQueues.get(i);
            Task t = q.peek();
            //if (t != null) t.RanOnce = false;
            q.clear();
        }
        while (asyncQueues.size() > 1)
            asyncQueues.remove(asyncQueues.size() - 1);
    }

    public void clear() {
        asyncQueues.get(0).clear();
        clearAsyncQueues();
    }

    public void update() {
        if (IsSchedulerDone()) return;

        // update each queue
        for (int i = 0; i < asyncQueues.size(); i++) {
            Queue<Task> q = asyncQueues.get(i);
            if (q.isEmpty()) continue;

            Task t = q.peek();
            //if (t.Run()) q.poll();
        }

        // remove empty finished async queues
        for (int i = asyncQueues.size() - 1; i >= 1; i--) {
            if (asyncQueues.get(i).isEmpty())
                asyncQueues.remove(i);
        }
    }

    // Main s
    public boolean IsSchedulerDone() {
        return asyncQueues.get(0).isEmpty() && asyncQueues.size() == 1;
    }
}
