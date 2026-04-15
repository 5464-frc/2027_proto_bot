package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartLogger extends SubsystemBase {

    public static class loggingItem {
        String name = "unregistered";
        List<String> data = new ArrayList<>();
        int maxLen = 0;

        public loggingItem(String name, int maxLen) {
            this.name = name;
            this.maxLen = maxLen;
        }

        public void pushValue(String data) {
            this.data.add(0, data);
            if (this.data.size() > this.maxLen) {
                this.data.remove(maxLen - 1);
            }
        }

        public void smartdashboardHook() {
            String[] output = new String[this.data.size()];

            for (int i = 0; i < this.data.size(); i++) {
                output[i] = this.data.get(i);
            }
            SmartDashboard.putStringArray(this.name, output);
        }
    }
}
