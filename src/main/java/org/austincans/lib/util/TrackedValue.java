package org.austincans.lib.util;

public class TrackedValue<T> {
    private T value;
    private boolean changed;

    public void set(T newValue) {
        changed = !value.equals(newValue);
        value = newValue;
    }

    public T get() {
        changed = false;
        return value;
    }

    public boolean hasChanged() {
        return changed;
    }
}
