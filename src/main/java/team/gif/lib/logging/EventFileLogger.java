package team.gif.lib.logging;

import java.io.IOException;

/**
 * @author Patrick Ubelhor
 * @since 3/17/2022
 */
public class EventFileLogger extends FileLogger {

    private static final String EVENT_LOG_PREFIX = "events";


    /**
     * Constructs a <code>EventFileLogger</code>.
     */
    public EventFileLogger() {
        super(EVENT_LOG_PREFIX);
    }

    public void addEvent(String mode, String event) {
        if (fw == null) {
            return;
        }

        String line = String.format("%s,[%s],%s", getCurrentTime(), mode, event);

        try {
            fw.append(line).append("\n");
        } catch (IOException e) {
            System.err.println("Failed to run event logger");
            System.err.println(e.getMessage());
            e.printStackTrace();
        }

        flush();
    }
}
