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

        # Check if we've reached the waypoint
        distance_to_waypoint = self._calculate_distance_squared(current_position, self.waypoint)
        if distance_to_waypoint <= self.acceptance_radius * self.acceptance_radius:
            self.at_waypoint = True

        # Determine our target: waypoint first, then landing pad
        if self.closest_landing_pad is None:
            # No landing pads, just use waypoint
            target = self.waypoint
        elif self.at_waypoint:
            # We've been to waypoint, now go to landing pad
            target = self.closest_landing_pad
        else:
            # Still going to waypoint first
            target = self.waypoint

        # Handle different drone statuses
        if current_status == drone_status.DroneStatus.HALTED:
            # Calculate distance to current target
            distance_squared = self._calculate_distance_squared(current_position, target)

            if distance_squared <= self.acceptance_radius * self.acceptance_radius:
                # Close enough to target - land the drone
                command = commands.Command.create_land_command()
            else:
                # Move towards current target
                dx = target.location_x - current_position.location_x
                dy = target.location_y - current_position.location_y
                command = commands.Command.create_set_relative_destination_command(
                    relative_x=dx, relative_y=dy
                )

        # ============
        # ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
        # ============

        return command

        # Helper methods below

    def _calculate_distance_squared(
        self, pos1: location.Location, pos2: location.Location
    ) -> float:
        # Calculate squared distance between two locations
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
            distance_squared = self._calculate_distance_squared(self.waypoint, landing_pad)

            if distance_squared < min_distance_squared:
                min_distance_squared = distance_squared
                closest_pad = landing_pad

        return closest_pad
