/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Manages persistent robot configuration data on the RoboRIO.
 * Handles versioning of configuration files using JSON serialization and
 * maintains a symbolic link to the current active configuration.
 */
public class ConfigManager {
    private static ConfigManager INSTANCE = null;
    private final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    private final Path configDir;
    private final Path currentLink;

    private Path currVerFile;
    private RobotConfig currConfig;

    private boolean ready = false;
    private final List<Runnable> readyListeners = new ArrayList<>();
    private final List<Tuneable<?>> tuneables = new ArrayList<>();

    /**
     * @return The singleton instance of the ConfigManager.
     */
    public static synchronized ConfigManager getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ConfigManager(
                    Path.of(Filesystem.getDeployDirectory().toPath().toString(), "../configs/"));
        }

        return INSTANCE;
    }

    /**
     * Gets or creates an instance for testing purposes using a specific directory.
     * * @param configDir The path to the test configuration directory.
     * @return An instance of ConfigManager.
     */
    public static synchronized ConfigManager getTestInstance(Path configDir) {
        if (INSTANCE == null) {
            INSTANCE = new ConfigManager(configDir);
            INSTANCE.init();
        }

        return INSTANCE;
    }

    /**
     * Registers a component that relies on configuration values.
     * @param t The {@link Tuneable} component to register.
     */
    public void registerTunable(Tuneable<?> t) {
        tuneables.add(t);
    }

    /**
     * Executes a task once the configuration is initialized.
     * @param r The runnable task to execute.
     */
    public synchronized void onReady(Runnable r) {
        if (ready) {
            r.run();
        } else {
            readyListeners.add(r);
        }
    }

    /**
     * Initializes all registered {@link Tuneable} components and triggers ready listeners.
     */
    public void init() {
        if (ready) return;
        ready = true;

        System.out.println(tuneables.size() + " Tuneable values");
        for (int i = 0; i < tuneables.size(); i++) {
            tuneables.get(i).initFromConfig();
        }

        System.out.println(readyListeners);
        readyListeners.forEach(Runnable::run);
        readyListeners.clear();

        Thread thread = new Thread(() -> {
            while (true) {
                tuneablePerodic();
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        thread.start();
    }

    private void tuneablePerodic() {
        for (int i = 0; i < tuneables.size(); i++) {
            tuneables.get(i).periodic();
        }
    }

    /**
     * Retrieves a value from the currently loaded configuration.
     * * @param key The key to look up.
     * @return The value associated with the key.
     */
    public Object get(String key) {
        return this.currConfig.values.get(key);
    }

    /**
     * Sets a value in the current configuration and persists the changes.
     * * @param key The key to store.
     * @param value The value to associate with the key.
     */
    public synchronized void set(String key, Object value) {
        this.currConfig.values.put(key, value);
        this.saveCurrentVersion();
    }

    /**
     * Checks if a key exists in the current configuration.
     * @param key The key to check.
     * @return True if the key exists.
     */
    public boolean contains(String key) {
        return this.currConfig.values.containsKey(key);
    }

    /**
     * Prints the current configuration and file structure state to console for debugging.
     */
    public void debug() {
        System.out.println(this.currConfig.toString());
        System.out.println(this.currVerFile.toString());
        System.out.println(this.currentLink.toString());

        try (BufferedReader reader = Files.newBufferedReader(this.currVerFile)) {
            while (true) {
                String line = reader.readLine();
                if (line == null) break;
                System.out.println(line);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Initializes the manager and loads the current configuration.
     * Resolves the "current" symbolic link or creates a new configuration if none exists.
     */
    private ConfigManager(Path configDir) {
        this.configDir = configDir;
        this.currentLink = configDir.resolve("current.json");
        try {
            Files.createDirectories(this.configDir);

            if (Files.isSymbolicLink(currentLink)) {
                this.currVerFile = Files.readSymbolicLink(currentLink);
                if (!this.currVerFile.isAbsolute()) {
                    this.currVerFile = configDir.resolve(this.currVerFile);
                }
                this.loadFromFile(this.currVerFile);
            } else {
                this.currConfig = new RobotConfig();
                currConfig.version = 1;
                this.saveNewVersion();
            }
        } catch (IOException e) {
            throw new RuntimeException("Failed to initialize config manager", e);
        }
    }

    /**
     * Saves the current state to a versioned JSON file.
     * @param version The version number to save as.
     */
    public synchronized void saveVersion(int version) {
        try {
            this.currConfig.version = version;
            Path versionFile = configDir.resolve("v" + version + ".json");
            Path tempFile = configDir.resolve("v" + version + ".json.tmp");

            try (Writer writer = Files.newBufferedWriter(tempFile)) {
                gson.toJson(this.currConfig, writer);
            }

            Files.move(tempFile, versionFile, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Saves the current configuration to the existing active version file.
     */
    public void saveCurrentVersion() {
        this.saveVersion(this.currConfig.version);
    }

    /**
     * Saves the configuration as a new version and updates the "current" symbolic link.
     */
    public void saveNewVersion() {
        int version = getNextVersionNumber();
        this.saveVersion(version);
        try {
            Path versionFile = configDir.resolve("v" + version + ".json");
            if (Files.exists(currentLink)) {
                Files.delete(currentLink);
            }

            Files.createSymbolicLink(currentLink, versionFile.getFileName());
            currVerFile = versionFile;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Loads an existing version and updates the "current" symbolic link.
     * @param version The version number to load.
     */
    public void loadVersion(int version) {
        try {
            Path versionFile = configDir.resolve("v" + version + ".json");
            if (!Files.exists(versionFile)) {
                System.err.println("Version " + version + " does not exist.");
                return;
            }

            loadFromFile(versionFile);

            if (Files.exists(currentLink)) {
                Files.delete(currentLink);
            }
            Files.createSymbolicLink(currentLink, versionFile.getFileName());
            this.currVerFile = versionFile;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Retrieves an array of all available configuration versions.
     * @return An array of version integers.
     */
    public int[] getVersions() {
        try (var files = Files.list(configDir)) {
            return files.map(Path::getFileName)
                    .map(Path::toString)
                    .filter(name -> name.matches("v\\d+\\.json"))
                    .mapToInt(name -> Integer.parseInt(name.replaceAll("\\D", "")))
                    .sorted()
                    .toArray();
        } catch (IOException e) {
            e.printStackTrace();
            return new int[0];
        }
    }

    /**
     * Loads configuration data from a specific file.
     * @param file The path to the JSON file.
     */
    private void loadFromFile(Path file) {
        try (Reader reader = Files.newBufferedReader(file)) {
            this.currConfig = gson.fromJson(reader, RobotConfig.class);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Determines the next available version number based on file naming conventions.
     * @return The next version number.
     */
    private int getNextVersionNumber() {
        try (var files = Files.list(configDir)) {
            return files.map(Path::getFileName)
                            .map(Path::toString)
                            .filter(name -> name.matches("v\\d+\\.json"))
                            .mapToInt(name -> Integer.parseInt(name.replaceAll("\\D", "")))
                            .max()
                            .orElse(0)
                    + 1;
        } catch (IOException e) {
            return 1;
        }
    }

    /**
     * A wrapper class to store configuration data versions.
     */
    public class RobotConfig {
        public int version;
        public ConcurrentHashMap<String, Object> values = new ConcurrentHashMap<>();

        @Override
        public String toString() {
            return String.format("RobotConfig{version=%s,config=%s}", version, values.toString());
        }
    }
}
