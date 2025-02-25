import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# from rovr_interfaces.action import DigLocation
from rovr_control.costmap_2d import PyCostmap2D
from rovr_interfaces.srv import DigLocation

class DigLocationFinder(Node):
    def __init__(self):
        super().__init__("dig_location_finder")
        # self._action_server = ActionServer(
        #     self,
        #     DigLocation,
        #     "dig_location",
        #     self.execute_callback,
        #     cancel_callback=self.cancel_callback,
        # )

        self.srv = self.create_service(DigLocation, 'find_dig_location', self.find_dig_location_callback)

        self.absolute_max_dig_cost = self.declare_parameter("absolute_max_dig_cost", 200).value
        self.max_dig_cost = self.declare_parameter("max_dig_cost", 100).value
        # Ignore the 1 in here. It breaks if you default to empty list (it thinks its a byte array)
        self.all_dig_locations = self.declare_parameter("all_dig_locations", [1]).value
        
        # ROS doesn't like nested lists, so the config file has to be flattened.
        # This unflattens that list
        self.all_dig_locations = [(self.all_dig_locations[i], self.all_dig_locations[i+1]) for i in range(0, len(self.all_dig_locations), 2)]

        self.potential_dig_locations = self.all_dig_locations.copy()

    def find_dig_location_callback(self, request, response):
        coords = self.getDigLocation()
        response.x, response.y = coords[0], coords[1] if coords else None
        return response
        
    def updatePotentialDigLocations(self):
        try:
            costmap = PyCostmap2D(self.nav2.getGlobalCostmap())
            robot_width, robot_height = (0.5, 0.5)
            for location in self.potential_dig_locations:
                # dig_cost = maximum cost of the cells that the robot will dig
                dig_cost = costmap.getDigCost(location[0], location[1], robot_width, robot_height)
                if dig_cost >= self.max_dig_cost:
                    self.potential_dig_locations.remove(location)
            
        except:
            self.get_logger().error("Error in updatePotentialDigLocations")

    def getDigLocation(self):
        self.updatePotentialDigLocations()
        # If there are no potential dig locations, reset the potential dig locations and increase the max dig cost
        # if the max dig cost is > absolute max dig cost, return None
        if len(self.potential_dig_locations) == 0:
            self.potential_dig_locations = self.all_dig_locations.copy()
            self.max_dig_cost += 10
            if self.max_dig_cost >= self.absolute_max_dig_cost:
                return None
            return self.getDigLocation()        
        return self.potential_dig_locations[0]
    
def main(args=None):
    rclpy.init(args=args)
    dig_location_finder = DigLocationFinder()
    rclpy.spin(dig_location_finder)
    dig_location_finder.destroy_node()
    rclpy.shutdown()