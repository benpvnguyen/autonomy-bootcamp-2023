"""
BOOTCAMPERS TO COMPLETE.

Travel to designated waypoint.
"""

from .. import commands
from .. import drone_report

# Disable for bootcamp use
# pylint: disable-next=unused-import
from .. import drone_status
from .. import location
from ..private.decision import base_decision


# Disable for bootcamp use
# No enable
# pylint: disable=duplicate-code,unused-argument


class DecisionSimpleWaypoint(base_decision.BaseDecision):
    """
    Travel to the designed waypoint.
    """

    def __init__(self, waypoint: location.Location, acceptance_radius: float) -> None:
        """
        Initialize all persistent variables here with self.
        """
        self.waypoint = waypoint
        print(f"Waypoint: {waypoint}")

        self.acceptance_radius = acceptance_radius

        # ============
        # ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
        # ============
        self.boundary_min = -60.0
        self.boundary_max = 60.0
        # Track the closest landing pad (calculate once when we reach waypoint area)
        self.closest_landing_pad = None
        self.closest_landing_pad_calculated = False

        # Track our current target (waypoint first, then closest landing pad)
        self.current_target = None
        # ============
        # ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
        # ============

    def run(
        self, report: drone_report.DroneReport, landing_pad_locations: "list[location.Location]"
    ) -> commands.Command:
        """
        Make the drone fly to the waypoint.

        You are allowed to create as many helper methods as you want,
        as long as you do not change the __init__() and run() signatures.

        This method will be called in an infinite loop, something like this:

        ```py
        while True:
            report, landing_pad_locations = get_input()
            command = Decision.run(report, landing_pad_locations)
            put_output(command)
        ```
        """
        # Default command
        command = commands.Command.create_null_command()

        # ============
        # ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
        # ============
        # Get current drone status and position
        current_status = report.status
        current_position = report.position

        # If we haven't calculated the closest landing pad yet and we have landing pad data
        if not self.closest_landing_pad_calculated and landing_pad_locations:
            # Calculate closest landing pad to waypoint (not current position)
            self.closest_landing_pad = self._find_closest_landing_pad(self.waypoint, landing_pad_locations)
            self.closest_landing_pad_calculated = True
            print(f"Closest landing pad to waypoint: {self.closest_landing_pad}")

        # Determine our current target
        if self.closest_landing_pad is None:
            # No landing pads available, just go to waypoint
            self.current_target = self.waypoint
        else:
            # Check if waypoint and closest landing pad are the same location
            if self._calculate_distance_squared(self.waypoint, self.closest_landing_pad) < 0.001:
                # Waypoint and landing pad are essentially the same, go directly there
                self.current_target = self.closest_landing_pad
            else:
                # Different locations - go to waypoint first, then landing pad
                distance_to_waypoint = self._calculate_distance(current_position, self.waypoint)

                if distance_to_waypoint <= self.acceptance_radius:
                    # We've reached the waypoint area, now target the landing pad
                    self.current_target = self.closest_landing_pad
                else:
                    # Still need to reach waypoint first
                    self.current_target = self.waypoint

        # Calculate distance to current target
        distance_to_target = self._calculate_distance(current_position, self.current_target)

        # Handle different drone statuses
        if current_status == drone_status.DroneStatus.HALTED:
            # Drone is halted - this is when we make movement decisions

            # Check if we're within acceptance radius of our current target
            if distance_to_target <= self.acceptance_radius:
                # If our current target is the landing pad, land
                if self.current_target == self.closest_landing_pad:
                    command = commands.Command.create_land_command()
                else:
                    # We reached waypoint, now switch target to landing pad for next iteration
                    # Send null command to let the target switching logic work next iteration
                    command = commands.Command.create_null_command()
            else:
                # We need to move towards the current target
                relative_movement = self._calculate_relative_movement(current_position)

                # Only create movement command if we have valid movement
                if relative_movement is not None:
                    command = commands.Command.create_set_relative_destination_command(
                        relative_x=relative_movement.location_x,
                        relative_y=relative_movement.location_y
                    )
                else:
                    # If we can't move (e.g., at boundary), try to land
                    command = commands.Command.create_land_command()

        elif current_status == drone_status.DroneStatus.MOVING:
            # Drone is moving, let it continue with null command
            command = commands.Command.create_null_command()

        elif current_status == drone_status.DroneStatus.LANDED:
            # Drone has landed, simulation ending
            command = commands.Command.create_null_command()

        # ============
        # ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
        # ============

        return command

    # Helper methods below
    def _calculate_distance(self, pos1: location.Location, pos2: location.Location) -> float:
        """
        Calculate Euclidean distance between two locations.
        """
        return (self._calculate_distance_squared(pos1, pos2)) ** 0.5

    def _calculate_distance_squared(self, pos1: location.Location, pos2: location.Location) -> float:
        """
        Calculate squared Euclidean distance between two locations.
        Avoids expensive square root calculation when only comparing distances.
        """
        dx = pos2.location_x - pos1.location_x
        dy = pos2.location_y - pos1.location_y
        return dx * dx + dy * dy

    def _find_closest_landing_pad(self, reference_point: location.Location,
                                landing_pad_locations: "list[location.Location]") -> location.Location:
        """
        Find the landing pad closest to the reference point (waypoint).
        Uses squared distance to avoid expensive square root calculations.
        """
        if not landing_pad_locations:
            return None

        closest_pad = None
        min_distance_squared = float('inf')  # Initialize with infinity

        for landing_pad in landing_pad_locations:
            # Use squared distance to avoid square root calculation
            distance_squared = self._calculate_distance_squared(reference_point, landing_pad)

            if distance_squared < min_distance_squared:
                min_distance_squared = distance_squared
                closest_pad = landing_pad

        return closest_pad

    def _calculate_relative_movement(self, current_pos: location.Location) -> location.Location:
        """
        Calculate relative movement towards current target, ensuring it stays within boundary.

        Returns:
            Location object with relative movement, or None if no valid movement
        """
        # Use current target (either waypoint or closest landing pad)
        target = self.current_target
        if target is None:
            target = self.waypoint  # Fallback to waypoint

        # Calculate direct vector to target
        dx = target.location_x - current_pos.location_x
        dy = target.location_y - current_pos.location_y

        # If we're very close, no movement needed
        if abs(dx) < 0.001 and abs(dy) < 0.001:
            return None

        # Calculate target position after movement
        target_x = current_pos.location_x + dx
        target_y = current_pos.location_y + dy

        # Clamp to flight boundary
        target_x = max(self.boundary_min, min(self.boundary_max, target_x))
        target_y = max(self.boundary_min, min(self.boundary_max, target_y))

        # Calculate final relative movement based on clamped target
        final_dx = target_x - current_pos.location_x
        final_dy = target_y - current_pos.location_y

        # If no movement would occur, return None
        if abs(final_dx) < 0.001 and abs(final_dy) < 0.001:
            return None

        # Return relative movement as Location object
        return location.Location(final_dx, final_dy)

    def _is_within_boundary(self, pos: location.Location) -> bool:
        """
        Check if position is within flight boundary.
        """
        return (self.boundary_min <= pos.location_x <= self.boundary_max and
                self.boundary_min <= pos.location_y <= self.boundary_max)
