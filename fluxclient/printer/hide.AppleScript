tell application "System Events"
    tell process "slic3r"
        set visible to false
    end tell
end tell

tell application "System Events"
    tell process "FLUX Studio"
        set frontmost to true
        set frontmost to false
    end tell
end tell
