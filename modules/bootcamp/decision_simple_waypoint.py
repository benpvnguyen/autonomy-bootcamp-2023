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
        # no extra variables needed
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
        current_status = report.status
        current_position = report.position

        # Only make decisions when drone is halted
        if current_status == drone_status.DroneStatus.HALTED:
            # Calculate distance to waypoint
            distance_squared = self._calculate_distance_squared(current_position, self.waypoint)

            if distance_squared <= self.acceptance_radius * self.acceptance_radius:
                # Close enough - land the drone
                command = commands.Command.create_land_command()
            else:
                # Move towards waypoint
                dx = self.waypoint.location_x - current_position.location_x
                dy = self.waypoint.location_y - current_position.location_y
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
        # Calculates squared distance between two locations
        dx = pos2.location_x - pos1.location_x
        dy = pos2.location_y - pos1.location_y
        return dx * dx + dy * dy
