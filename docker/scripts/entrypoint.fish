# entrypoint.fish
if not functions -q _ros2_fish_greeting
    bass source /opt/ros/jazzy/setup.bash
    fisher install kpbaks/ros2.fish
end

exec fish