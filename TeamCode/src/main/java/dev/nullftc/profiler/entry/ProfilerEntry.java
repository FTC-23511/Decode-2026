package dev.nullftc.profiler.entry;

public abstract class ProfilerEntry {
    private final String type;
    private final long startTime;
    private final long endTime;

    protected ProfilerEntry(String type, long startTime, long endTime) {
        this.type = type;
        this.startTime = startTime;
        this.endTime = endTime;
    }

    public String getType() { return type; }
    public long getStartTime() { return startTime; }
    public long getEndTime() { return endTime; }
    public long getDeltaTime() { return endTime - startTime; }

    public abstract String[] toCSVRow();
}
