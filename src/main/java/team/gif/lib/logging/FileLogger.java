package team.gif.lib.logging;

import edu.wpi.first.wpilibj.Timer;

import java.io.*;

/**
 * @author Patrick Ubelhor
 * @since 3/9/2022
 */
abstract class FileLogger implements Closeable, Flushable {

    private static final String EVENT_LOG_PREFIX = "events";
    private static final String FILE_EXTENSION = ".csv";

    protected final FileWriter fw;
    protected double initTime = 0; // TODO: share this value among loggers
    private String[] directories = {
        "/media/sda2/logs", // Samsung USB
        "/media/sdb1/logs",
        "/media/sda1/logs", // Our new USB
		"/home/lvuser/logs", // Local dir on roborio
    };


    /**
     * Constructs a <code>FileLogger</code>.
     */
    protected FileLogger(String filenamePrefix) {
        fw = createLogWriter(filenamePrefix);
    }

    protected FileWriter createLogWriter(String filePrefix) {
        File logDir = createLogDir(directories);
        if (logDir == null) { // If we couldn't create/find the dir, then abort
            return null;
        }

        // TODO: Use a file to track file numbers
        int nextFileNumber = getNextFileNumber(logDir);
        String nextFileName = filePrefix + nextFileNumber + FILE_EXTENSION;

        return createFileWriter(logDir, nextFileName);
    }

    private File createLogDir(String[] directories) {
        for (String location : directories) {
            File dir = new File(location);
            if (dir.exists() || dir.mkdir()) {
                return dir;
            }
        }

        System.err.println("Could not create log directory!");
        return null;
    }

    private int getNextFileNumber(File logDir) {
        String[] lognames = logDir.list((File dir, String name) -> name.startsWith(EVENT_LOG_PREFIX));

        if (lognames == null) {
            System.err.println("Failed to read log directory");
            return 0;
        }

        int max = 0;
        for (String logname : lognames) {
            String withoutExtension = logname.substring(0, logname.length() - FILE_EXTENSION.length());
            String rawNumber = withoutExtension.substring(EVENT_LOG_PREFIX.length());
            int number = Integer.parseInt(rawNumber);

            max = Math.max(max, number);
        }

        return max + 1;
    }

    private FileWriter createFileWriter(File logDir, String filename) {
        FileWriter fw;
        try {
            fw = new FileWriter(logDir.getPath() + "/" + filename);
        } catch (IOException e) {
            System.err.println("Failed to create " + filename);
            System.err.println(e.getMessage());
            e.printStackTrace();

            fw = null;
        }

        return fw;
    }

    protected final double getCurrentTime() {
        return Timer.getFPGATimestamp() - initTime;
    }

    protected String formatDouble(double number) {
        return String.format("%.2f", number);
    }

    protected String formatBoolean(boolean var) {
        return var ? "1" : "0";
    }

    public void init() {
        initTime = Timer.getFPGATimestamp();
    }

    /**
     * Flushes the contents of the internal buffer to the file.
     * <p>
     * If the robot shuts down before the buffer is flushed, any contents still
     * in the buffer will be lost. However, flushing the buffer is a <i>potentially</i>
     * expensive action, and should not be done on every control loop. The buffer
     * may flush itself at any time.
     */
    @Override
    public final void flush() {
        if (fw == null) {
            return;
        }

        try {
            fw.flush();
        } catch (IOException e) {
            System.err.println("Failed to flush logger");
            System.err.println(e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Closes the logger, flushing it first. Once the logger has been closed,
     * further attempts to write or flush will cause an IOException to be thrown.
     * Closing a previously closed logger has no effect.
     */
    @Override
    public final void close() {
        if (fw == null) {
            return;
        }

        try {
            fw.close();
        } catch (IOException e) {
            System.err.println("Failed to close logger");
            System.err.println(e.getMessage());
            e.printStackTrace();
        }
    }
}
