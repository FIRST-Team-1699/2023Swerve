package frc.robot.team1699.lib.auto.modes;

import java.util.ArrayList;
import frc.robot.team1699.lib.auto.events.Event;
import frc.robot.team1699.lib.auto.events.FollowTrajectory;
import frc.robot.team1699.subsystems.Drive;

public class DriveForward extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public DriveForward(Drive swerve) {
        events.add(new FollowTrajectory(null, swerve));

        i = 0;
    }

    public void initialize() {
        events.get(i).initialize();
    }

    public void run() {
        Event currentEvent = events.get(i);
        if(currentEvent.isFinished()) {
            currentEvent.finish();
            i++;
            events.get(i).initialize();
        } else {
            currentEvent.update();
        }
    }

    public boolean isFinished() {
        if(i > events.size()) {
            return true;
        }
        return false;
    }

    public void finish() {
        System.out.println("Done with automode");
    }
}
