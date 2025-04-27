import unittest
from unittest.mock import patch, MagicMock, AsyncMock, call
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import NavigateToPose
from rovr_interfaces.action import GoToDigLocation
from rovr_interfaces.srv import DigLocation
import math
import numpy as np

# Import the class being tested
from rovr_control.dig_location_finder import DigLocationFinder


class TestDigLocationFinder(unittest.TestCase):
    
    def setUp(self):
        # Initialize rclpy for tests
        rclpy.init()
        
        # Create mock for costmap
        self.mock_costmap = MagicMock()
        
        # Patch the PyCostmap2D class
        self.patcher = patch('rovr_control.dig_location_finder.PyCostmap2D')
        self.mock_costmap_class = self.patcher.start()
        self.mock_costmap_instance = self.mock_costmap_class.return_value
        
        # Create the node under test
        self.node = DigLocationFinder()
        
        # Override default values for testing
        self.node.all_dig_locations = [(0.6, 0.37), (0.6, 1.1), (1.8, 0.37)]
        self.node.potential_dig_locations = self.node.all_dig_locations.copy()
        
    def tearDown(self):
        # Clean up node and rclpy
        self.node.destroy_node()
        self.patcher.stop()
        rclpy.shutdown()
    
    def test_initialization(self):
        """Test that the node initializes properly."""
        # Check if services and action servers are created
        self.assertIsNotNone(self.node._action_server)
        self.assertIsNotNone(self.node.nav2_client)
        self.assertIsNotNone(self.node.get_costmap_global_srv)
        
        # Check default values
        self.assertEqual(self.node.footprint, (1.2, 0.75))
        self.assertEqual(self.node.absolute_max_dig_cost, 200)
        self.assertEqual(self.node.max_dig_cost, 100)
        
        # Check that dig locations were properly unpacked
        self.assertIsInstance(self.node.all_dig_locations, list)
        self.assertIsInstance(self.node.all_dig_locations[0], tuple)
        self.assertEqual(len(self.node.all_dig_locations[0]), 2)
    
    def test_get_robot_footprint(self):
        """Test that footprint is properly calculated from polygon message."""
        msg = PolygonStamped()
        # Create a rectangular footprint (1.0 x 0.5)
        points = [
            Point32(x=0.5, y=0.25, z=0.0),
            Point32(x=0.5, y=-0.25, z=0.0),
            Point32(x=-0.5, y=-0.25, z=0.0),
            Point32(x=-0.5, y=0.25, z=0.0)
        ]
        msg.polygon.points = points
        
        # Mock the destroy_subscription method
        self.node.destroy_subscription = MagicMock()
        
        # Call the method
        self.node.get_robot_footprint(msg)
        
        # Check the footprint was updated correctly
        self.assertAlmostEqual(self.node.footprint[0], 1.0, delta=0.01)
        self.assertAlmostEqual(self.node.footprint[1], 0.5, delta=0.01)
        
        # Check destroy_subscription was called
        self.node.destroy_subscription.assert_called_once_with(self.node.footprint_sub)
    
    def test_get_goal_pose(self):
        """Test that goal pose message is created correctly."""
        x, y, yaw = 1.0, 2.0, math.pi/2
        
        goal_msg = self.node.get_goal_pose(x, y, yaw)
        
        # Check position
        self.assertEqual(goal_msg.pose.pose.position.x, x)
        self.assertEqual(goal_msg.pose.pose.position.y, y)
        
        # Check orientation (quaternion for 90 degrees rotation)
        self.assertAlmostEqual(goal_msg.pose.pose.orientation.x, 0.0, delta=0.001)
        self.assertAlmostEqual(goal_msg.pose.pose.orientation.y, 0.0, delta=0.001)
        self.assertAlmostEqual(goal_msg.pose.pose.orientation.z, 0.7071, delta=0.001)
        self.assertAlmostEqual(goal_msg.pose.pose.orientation.w, 0.7071, delta=0.001)
        
        # Check frame_id
        self.assertEqual(goal_msg.pose.header.frame_id, "map")
    
    def test_get_dig_location(self):
        """Test the getDigLocation method."""
        # Test normal case
        location = self.node.getDigLocation()
        self.assertEqual(location, (0.6, 0.37))
        
        # Test empty potential dig locations
        self.node.potential_dig_locations = []
        self.node.max_dig_cost = 190  # Less than absolute max
        
        # Mock getDigLocation to avoid infinite recursion
        original_method = self.node.getDigLocation
        self.node.getDigLocation = MagicMock(return_value=(0.6, 0.37))
        
        location = original_method()
        
        # Check that potential dig locations were reset
        self.assertEqual(self.node.potential_dig_locations, self.node.all_dig_locations)
        # Check that max dig cost was increased
        self.assertEqual(self.node.max_dig_cost, 200)
        
        # Restore the original method
        self.node.getDigLocation = original_method
        
        # Test exceeding absolute max dig cost
        self.node.potential_dig_locations = []
        self.node.max_dig_cost = 200  # Equal to absolute max
        
        location = self.node.getDigLocation()
        self.assertIsNone(location)
    
    def test_find_dig_location_callback(self):
        """Test the service callback for finding dig locations."""
        # Mock the getDigLocation method
        self.node.getDigLocation = MagicMock(return_value=(1.0, 2.0))
        
        # Create request and response objects
        request = DigLocation.Request()
        response = DigLocation.Response()
        
        # Call the callback
        result = self.node.find_dig_location_callback(request, response)
        
        # Check that the response was filled correctly
        self.assertTrue(result.success)
        self.assertEqual(result.x, 1.0)
        self.assertEqual(result.y, 2.0)
        
        # Test failure case
        self.node.getDigLocation = MagicMock(return_value=None)
        response = DigLocation.Response()
        
        result = self.node.find_dig_location_callback(request, response)
        
        self.assertFalse(result.success)
    
    @patch('rclpy.spin_until_future_complete')
    def test_get_global_costmap(self, mock_spin):
        """Test retrieving the global costmap."""
        # Create mock future and result
        mock_future = MagicMock()
        mock_future.result.return_value = GetCostmap.Response(map=OccupancyGrid())
        
        # Mock the service call
        self.node.get_costmap_global_srv.call_async = MagicMock(return_value=mock_future)
        self.node.get_costmap_global_srv.wait_for_service = MagicMock(return_value=True)
        
        result = self.node.getGlobalCostmap()
        
        # Check that the service was called
        self.node.get_costmap_global_srv.call_async.assert_called_once()
        # Check that we waited for the future
        mock_spin.assert_called_once()
        # Check that we returned the costmap
        self.assertIsInstance(result, OccupancyGrid)
        
        # Test failure case
        mock_future.result.return_value = None
        self.node.error = MagicMock()  # Mock error logging
        
        result = self.node.getGlobalCostmap()
        
        self.assertIsNone(result)
        self.node.error.assert_called_once()
    
    def test_update_potential_dig_locations(self):
        """Test updating potential dig locations based on costmap."""
        # Mock getGlobalCostmap
        mock_costmap = OccupancyGrid()
        self.node.getGlobalCostmap = MagicMock(return_value=mock_costmap)
        
        # Configure mock costmap instance
        self.mock_costmap_instance.getDigCost.side_effect = [50, 110, 90]
        
        # Call the method
        self.node.updatePotentialDigLocations()
        
        # Check that the second location was removed (cost 110 > max_dig_cost 100)
        expected_locations = [(0.6, 0.37), (1.8, 0.37)]
        self.assertEqual(self.node.potential_dig_locations, expected_locations)
        
        # Test exception handling
        self.node.getGlobalCostmap.side_effect = Exception("Test exception")
        self.node.get_logger = MagicMock()
        self.node.get_logger.return_value.error = MagicMock()
        
        # This should not raise an exception
        self.node.updatePotentialDigLocations()
        
        # Check that error was logged
        self.node.get_logger.return_value.error.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_drive_to_dig_location_success(self):
        """Test successful navigation to dig location."""
        # Mock getDigLocation
        self.node.getDigLocation = MagicMock(return_value=(1.0, 2.0))
        
        # Mock get_goal_pose
        mock_goal = NavigateToPose.Goal()
        self.node.get_goal_pose = MagicMock(return_value=mock_goal)
        
        # Mock navigation client
        self.node.nav2_client = MagicMock()
        
        # Create mock goal handle
        goal_handle = MagicMock()
        
        # Set up the navigation action result
        mock_goal_response = MagicMock()
        mock_goal_response.accepted = True
        
        mock_result_future = MagicMock()
        mock_result = MagicMock()
        mock_result.status = 4  # SUCCESS
        mock_result_future.result.return_value = mock_result
        
        mock_goal_response.get_result_async.return_value = mock_result_future
        self.node.nav2_client.send_goal_async.return_value = AsyncMock(return_value=mock_goal_response)()
        
        # Call the method
        result = await self.node.drive_to_dig_location(goal_handle)
        
        # Check that navigation goal was sent
        self.node.nav2_client.send_goal_async.assert_called_once_with(mock_goal)
        
        # Check result
        self.assertIsInstance(result, GoToDigLocation.Result)
    
    @pytest.mark.asyncio
    async def test_drive_to_dig_location_failure(self):
        """Test navigation failure cases."""
        # Test case: No dig location available
        self.node.getDigLocation = MagicMock(return_value=None)
        goal_handle = MagicMock()
        
        result = await self.node.drive_to_dig_location(goal_handle)
        
        goal_handle.abort.assert_called_once()
        
        # Test case: Navigation goal rejected
        self.node.getDigLocation = MagicMock(return_value=(1.0, 2.0))
        mock_goal = NavigateToPose.Goal()
        self.node.get_goal_pose = MagicMock(return_value=mock_goal)
        
        mock_goal_response = MagicMock()
        mock_goal_response.accepted = False
        self.node.nav2_client.send_goal_async.return_value = AsyncMock(return_value=mock_goal_response)()
        
        goal_handle = MagicMock()
        self.node.get_logger = MagicMock()
        
        result = await self.node.drive_to_dig_location(goal_handle)
        
        goal_handle.abort.assert_called_once()


if __name__ == '__main__':
    unittest.main()