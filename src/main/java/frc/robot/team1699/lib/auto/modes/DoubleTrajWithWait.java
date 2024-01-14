package frc.robot.team1699.lib.auto.modes;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import java.util.ArrayList;
import frc.robot.team1699.lib.auto.events.Event;
import frc.robot.team1699.lib.auto.events.FollowTrajectory;
import frc.robot.team1699.lib.auto.events.WaitEvent;
import frc.robot.team1699.subsystems.Drive;

public class DoubleTrajWithWait extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public DoubleTrajWithWait(PathPlannerTrajectory trajectoryOne, PathPlannerTrajectory trajectoryTwo, double waitTime, Drive swerve) {
        events = new ArrayList<Event>();
        events.add(new FollowTrajectory(trajectoryOne, swerve));
        events.add(new WaitEvent(waitTime));
        events.add(new FollowTrajectory(trajectoryTwo, swerve));

        i = 0;
    }


    @Override
    public void initialize() {
        events.get(i).initialize();
    }

    @Override
    public void run() {
        if(i < events.size()) {
            Event currentEvent = events.get(i);
            if(currentEvent.isFinished()) {
                currentEvent.finish();
                i++;
                if(i < events.size()) {
                    events.get(i).initialize();
                }
            } else {
                currentEvent.update();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if(i >= events.size()) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {}
    
}
