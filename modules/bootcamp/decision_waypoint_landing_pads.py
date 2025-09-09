"""
BOOTCAMPERS TO COMPLETE.

Travel to designated waypoint and then land at a nearby landing pad.
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


class DecisionWaypointLandingPads(base_decision.BaseDecision):
    """
    Travel to the designed waypoint and then land at the nearest landing pad.
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

        # boundaries
        self.boundary_min = -60.0
        self.boundary_max = 60.0

        # Track closest landing pad (calculate once)
        self.closest_landing_pad = None
        self.landing_pad_found = False

        # Track waypoint
        self.at_waypoint = False
        # ============
        # ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
        # ============

    def run(
        self, report: drone_report.DroneReport, landing_pad_locations: "list[location.Location]"
    ) -> commands.Command:
        """
        Make the drone fly to the waypoint and then land at the nearest landing pad.

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

        # Get drone status and position
        current_status = report.status
        current_position = report.position

        # Find closest landing pad
        if not self.landing_pad_found and landing_pad_locations:
            self.closest_landing_pad = self._find_closest_landing_pad(landing_pad_locations)
            self.landing_pad_found = True
            print(f"Closest landing pad: {self.closest_landing_pad}")

        # Determine our target: waypoint first, then landing pad
        distance_to_waypoint = self._calculate_distance(current_position, self.waypoint)

        if distance_to_waypoint <= self.acceptance_radius:
            self.at_waypoint = True

        # Choose target
        if self.closest_landing_pad is None:
            # No landing pads, just use waypoint
            target = self.waypoint
        elif self._calculate_distance(self.waypoint, self.closest_landing_pad) <= 0.1:
            # If Waypoint and landing pad are same location, go directly to landing pad
            target = self.closest_landing_pad
        elif self.at_waypoint:
            # We've been to waypoint, now go to landing pad
            target = self.closest_landing_pad
        else:
            # Still going to waypoint first
            target = self.waypoint

        # Calculate distance to current target
        distance_to_target = self._calculate_distance(current_position, target)

        # Handle different drone statuses
        if current_status == drone_status.DroneStatus.HALTED:
            # Drone is halted

            # Check if we're within acceptance radius of target
            if distance_to_target <= self.acceptance_radius:
                # If target is landing pad, land. Otherwise continue.
                if target == self.closest_landing_pad:
                    command = commands.Command.create_land_command()
                elif self.closest_landing_pad is None:
                    # No landing pad exists, land at waypoint
                    command = commands.Command.create_land_command()
                else:
                    # At waypoint but need to go to landing pad next iteration
                    command = commands.Command.create_null_command()
            else:
                # We need to move towards the target
                relative_movement = self._calculate_relative_movement(current_position, target)

                # Only create movement command if we have valid movement
                if relative_movement is not None:
                    command = commands.Command.create_set_relative_destination_command(
                        relative_x=relative_movement.location_x,
                        relative_y=relative_movement.location_y,
                    )
                else:
                    # If we can't move, try to land
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
        # Calculate distance between two locations.

        dx = pos2.location_x - pos1.location_x
        dy = pos2.location_y - pos1.location_y
        return (dx * dx + dy * dy) ** 0.5

    def _calculate_distance_squared(
        self, pos1: location.Location, pos2: location.Location
    ) -> float:
        # Calculate squared distance

        dx = pos2.location_x - pos1.location_x
        dy = pos2.location_y - pos1.location_y
        return dx * dx + dy * dy

    def _find_closest_landing_pad(
        self, landing_pad_locations: "list[location.Location]"
    ) -> location.Location:
        # Find the landing pad closest to waypoint

        if not landing_pad_locations:
            return None

        closest_pad = None
        min_distance_squared = float("inf")  # Initialize with very large number

        for landing_pad in landing_pad_locations:
            # Use squared distance to avoid square root calculation (bonus)
            distance_squared = self._calculate_distance_squared(self.waypoint, landing_pad)

            if distance_squared < min_distance_squared:
                min_distance_squared = distance_squared
                closest_pad = landing_pad

        return closest_pad

    def _calculate_relative_movement(
        self, current_pos: location.Location, target: location.Location
    ) -> location.Location:

        # Calculate relative movement towards target, ensuring it stays within boundary.

        # Calculate direct vector to target
        dx = target.location_x - current_pos.location_x
        dy = target.location_y - current_pos.location_y

        # If we're very close, no movement needed
        if abs(dx) < 0.001 and abs(dy) < 0.001:
            return None

        # Calculate target position after movement
        target_x = current_pos.location_x + dx
        target_y = current_pos.location_y + dy

        # Clamp to flight boundary (copied from simple waypoint)
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
