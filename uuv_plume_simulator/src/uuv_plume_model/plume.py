# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Point32
from tf.transformations import quaternion_from_euler


class Plume(object):
    """
    Base class for plume model classes. Plume models should inherit this class
    and implement the update function to maintain a standard interface with the
    plume server. Each new inherited plume model must have an unique *LABEL*
    tag, since it will be used by the factory method to create plume models
    without the need to import all the plume model class files.
    """
    LABEL = ''

    def __init__(self, source_pos, n_points, start_time):
        assert n_points > 0, 'Number of points to generate the plume must be' \
            ' greater than zero'
        assert len(source_pos) == 3, 'Source position must have three elements'
        assert source_pos[2] <= 0, 'Source Z coordinate is above the water' \
            ' surface with respect to the ENU inertial frame'

        # Vector [N x 3] holding all the position vectors for each created
        # particle
        self._pnts = None
        # Time of creation vector will hold the time stamps of creation of each
        # particle in the plume
        self._time_creation = None

        # Limits for the bounding box where the plume particles can exist and
        # be generated
        self._x_lim = [-1e7, 1e7]
        self._y_lim = [-1e7, 1e7]
        self._z_lim = [-1e7, 0]

        # Position of the source of the plume
        self._source_pos = source_pos
        # Number of points in the point cloud
        self._n_points = n_points
        # Time stamp to be updated at each iteration
        self._t = 0.0
        # Time stamp for the creation of this plume model instance
        self._start_time = start_time
        # Time step between updates
        self._dt = 0.0
        # Current velocity vector to be updated by the plume server
        self._current_vel = np.zeros(3)

    @staticmethod
    def create_plume_model(tag, *args):
        """
        Factory function to create the plume model using the LABEL attribute
        as identifier.

        Parameters
        ----------
        tag: str
            Label of the plume model to be created.
        args:
            Input arguments for the specific plume model to be created.
        """
        for model in Plume.__subclasses__():
            if model.LABEL == tag:
                return model(*args)
        return None

    @property
    def source_pos(self):
        """
        Return the position vector with the Cartesian coordinates for the plume
        source.
        """
        return self._source_pos

    @source_pos.setter
    def source_pos(self, new_pos):
        assert isinstance(new_pos, list)
        assert len(new_pos) == 3

        # Reseting the plume
        self.reset_plume()
        self._source_pos = new_pos

        # Ensure that the new source position is inside the bounding box
        self._source_pos[0] = max(self._source_pos[0], self._x_lim[0])
        self._source_pos[0] = min(self._source_pos[0], self._x_lim[1])

        self._source_pos[1] = max(self._source_pos[1], self._y_lim[0])
        self._source_pos[1] = min(self._source_pos[1], self._y_lim[1])

        self._source_pos[2] = max(self._source_pos[2], self._z_lim[0])
        self._source_pos[2] = min(self._source_pos[2], self._z_lim[1])

    @property
    def x(self):
        """
        Return only the X coordinates for the particle positions.
        """
        if self._pnts is None:
            return None
        return self._pnts[:, 0]

    @property
    def x_lim(self):
        """
        Return the lower and upper limit for the bounding box on the X axis.
        """
        return self._x_lim

    @property
    def y(self):
        """
        Return only the Y coordinates for the particle positions.
        """
        if self._pnts is None:
            return None
        return self._pnts[:, 1]

    @property
    def time_of_creation(self):
        """Return the time of creation vector."""
        if self._pnts is None:
            return None
        return self._time_creation

    @property
    def y_lim(self):
        """
        Return the lower and upper limit for the bounding box on the Y axis.
        """
        return self._y_lim

    @property
    def z(self):
        """
        Return only the Z coordinates for the particle positions.
        """
        if self._pnts is None:
            return None
        return self._pnts[:, 2]

    @property
    def z_lim(self):
        """
        Return the lower and upper limit for the bounding box on the Z axis.
        """
        return self._z_lim

    @property
    def points(self):
        """
        Return list of [N x 3] position vectors for particles created.
        """
        return self._pnts

    @property
    def n_points(self):
        """
        Return the maximum number of points to be created by the plume model.
        """
        return self._n_points

    @property
    def num_particles(self):
        if self._pnts is None:
            return 0
        else:
            return self._pnts.shape[0]

    def set_n_points(self, n_points):
        """
        Set the maximum number of points to be created by the plume model.

        Parameters
        ----------
        n_points: int
            Number of particles (must be greater than zero).
        """
        if n_points > 0:
            self._n_points = n_points
            return True
        else:
            print 'Number of plume particle points must be positive'
            return False

    def update_current_vel(self, vel):
        """
        Update the current velocity vector.

        Parameters
        ----------
        vel: list or numpy.array
            Current velocity vector containing three elements :math:`(u, v, w)`.
        """
        self._current_vel = vel

    def set_x_lim(self, min_value, max_value):
        """
        Set the X limits for the plume bounding box. The bounding box is
        defined with respect to the ENU inertial frame.

        Parameters
        ----------
        min_value: float
            Lower limit for the bounding box over the X axis.
        max_value: float
            Upper limit for the bounding box over the X axis.
        """
        assert min_value < max_value, 'Limit interval is invalid'

        if self._source_pos[0] >= self._x_lim[0] and self._source_pos[0] <= self._x_lim[1]:
            self._x_lim[0] = min_value
            self._x_lim[1] = max_value
            return True
        else:
            print 'Plume source is outside of limits, ignoring new X limits'
            return False

    def set_y_lim(self, min_value, max_value):
        """
        Set the Y limits for the plume bounding box. The bounding box is
        defined with respect to the ENU inertial frame.

        Parameters
        ----------
        min_value: float
            Lower limit for the bounding box over the Y axis.
        max_value: float
            Upper limit for the bounding box over the Y axis.
        """
        assert min_value < max_value, 'Limit interval is invalid'

        if self._source_pos[1] >= self._y_lim[0] and self._source_pos[1] <= self._y_lim[1]:
            self._y_lim[0] = min_value
            self._y_lim[1] = max_value
            return True
        else:
            print 'Plume source is outside of limits, ignoring new Y limits'
            return False

    def set_z_lim(self, min_value, max_value):
        """
        Set the Z limits for the plume bounding box. The bounding box is
        defined with respect to the ENU inertial frame.

        Parameters
        ----------
        min_value: float
            Lower limit for the bounding box over the Z axis.
        max_value: float
            Upper limit for the bounding box over the Z axis.
        """
        assert min_value < 0, 'Minimum value must be lower than zero'
        assert min_value < max_value, 'Limit interval is invalid'

        if self._source_pos[2] >= self._z_lim[0] and \
            self._source_pos[2] <= self._z_lim[1]:
            self._z_lim[0] = min_value
            self._z_lim[1] = min(0, max_value)
            return True
        else:
            print 'Plume source is outside of limits, ignoring new Z limits'
            return False

    def reset_plume(self):
        """
        Reset point cloud and time of creating vectors.
        """
        self._pnts = None
        self._time_creation = None

    def get_contraints_filter(self):
        """
        Return a binary vector of N elements, N being current number of
        particles created. The i-th element is set to False if the i-th
        particle finds itself outside of the bounding box limits.
        """
        particle_filter = self._pnts[:, 0].flatten() >= self._x_lim[0]
        particle_filter = np.logical_and(
            particle_filter,
            self._pnts[:, 0].flatten() <= self._x_lim[1])

        particle_filter = np.logical_and(
            particle_filter,
            self._pnts[:, 1].flatten() >= self._y_lim[0])
        particle_filter = np.logical_and(
            particle_filter,
            self._pnts[:, 1].flatten() <= self._y_lim[1])

        particle_filter = np.logical_and(
            particle_filter,
            self._pnts[:, 2].flatten() >= self._z_lim[0])

        if self._z_lim[1] < 0:
            particle_filter = np.logical_and(
                particle_filter,
                self._pnts[:, 2].flatten() <= self._z_lim[1])
        return particle_filter

    def apply_constraints(self):
        """
        Truncate the position of the particle to the closest bounding box limit
        if the particle is positioned outside of the limits.
        """
        assert self._pnts is not None, 'Plume points have not been initialized'

        self._pnts[:, 0] = np.maximum(self._x_lim[0], self._pnts[:, 0])
        self._pnts[:, 0] = np.minimum(self._x_lim[1], self._pnts[:, 0])

        self._pnts[:, 1] = np.maximum(self._y_lim[0], self._pnts[:, 1])
        self._pnts[:, 1] = np.minimum(self._y_lim[1], self._pnts[:, 1])

        self._pnts[:, 2] = np.maximum(self._z_lim[0], self._pnts[:, 2])
        self._pnts[:, 2] = np.minimum(self._z_lim[1], self._pnts[:, 2])

    def set_plume_particles(self, t, x, y, z, time_creation):
        self._pnts = np.zeros(shape=(len(x), 3))
        self._time_creation = np.zeros(len(time_creation))

        self._time_creation = np.array(time_creation)
        self._time_creation -= np.max(time_creation)
        self._time_creation += t

        self._pnts[:, 0] = np.array(x)
        self._pnts[:, 1] = np.array(y)
        self._pnts[:, 2] = np.array(z)

    def get_point_cloud_as_msg(self):
        """
        Return a ROS point cloud sensor message with the points representing
        the plume's particles and one channel containing the time of creation
        for each particle.
        """
        if self._pnts is None or self._time_creation is None:
            return None

        pc = PointCloud()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = 'world'
        if self._pnts is None:
            return None
        pc.points = [Point32(x, y, z) for x, y, z in zip(self.x, self.y, self.z)]

        channel = ChannelFloat32()
        channel.name = 'time_creation'
        if self._time_creation is None:
            return None
        channel.values = self._time_creation.tolist()

        pc.channels.append(channel)
        return pc

    def get_markers(self):
        """
        Return a ROS marker array message structure with an sphere marker to
        represent the plume source, the bounding box and an arrow marker to
        show the direction of the current velocity if it's norm is greater
        than zero.
        """
        if self._pnts is None:
            return None

        marker_array = MarkerArray()

        # Generate marker for the source
        source_marker = Marker()
        source_marker.header.stamp = rospy.Time.now()
        source_marker.header.frame_id = 'world'

        source_marker.ns = 'plume'
        source_marker.id = 0
        source_marker.type = Marker.SPHERE
        source_marker.action = Marker.ADD
        source_marker.color.r = 0.35
        source_marker.color.g = 0.45
        source_marker.color.b = 0.89
        source_marker.color.a = 1.0

        source_marker.scale.x = 1.0
        source_marker.scale.y = 1.0
        source_marker.scale.z = 1.0
        source_marker.pose.position.x = self._source_pos[0]
        source_marker.pose.position.y = self._source_pos[1]
        source_marker.pose.position.z = self._source_pos[2]

        source_marker.pose.orientation.x = 0
        source_marker.pose.orientation.y = 0
        source_marker.pose.orientation.z = 0
        source_marker.pose.orientation.w = 1

        marker_array.markers.append(source_marker)

        # Creating the marker for the bounding box limits where the plume
        # particles are generated
        limits_marker = Marker()
        limits_marker.header.stamp = rospy.Time.now()
        limits_marker.header.frame_id = 'world'

        limits_marker.ns = 'plume'
        limits_marker.id = 1
        limits_marker.type = Marker.CUBE
        limits_marker.action = Marker.ADD

        limits_marker.color.r = 0.1
        limits_marker.color.g = 0.1
        limits_marker.color.b = 0.1
        limits_marker.color.a = 0.3

        limits_marker.scale.x = self._x_lim[1] - self._x_lim[0]
        limits_marker.scale.y = self._y_lim[1] - self._y_lim[0]
        limits_marker.scale.z = self._z_lim[1] - self._z_lim[0]

        limits_marker.pose.position.x = self._x_lim[0] + (self._x_lim[1] - self._x_lim[0]) / 2
        limits_marker.pose.position.y = self._y_lim[0] + (self._y_lim[1] - self._y_lim[0]) / 2
        limits_marker.pose.position.z = self._z_lim[0] + (self._z_lim[1] - self._z_lim[0]) / 2

        limits_marker.pose.orientation.x = 0
        limits_marker.pose.orientation.y = 0
        limits_marker.pose.orientation.z = 0
        limits_marker.pose.orientation.w = 0
        marker_array.markers.append(limits_marker)

        # Creating marker for the current velocity vector
        cur_vel_marker = Marker()
        cur_vel_marker.header.stamp = rospy.Time.now()
        cur_vel_marker.header.frame_id = 'world'

        cur_vel_marker.id = 1
        cur_vel_marker.type = Marker.ARROW

        if np.linalg.norm(self._current_vel) > 0:
            cur_vel_marker.action = Marker.ADD
            # Calculating the pitch and yaw angles for the current velocity
            # vector
            yaw = np.arctan2(self._current_vel[1], self._current_vel[0])
            pitch = np.arctan2(self._current_vel[2],
                np.sqrt(self._current_vel[0]**2 + self._current_vel[1]**2))
            qt = quaternion_from_euler(0, -pitch, yaw)

            cur_vel_marker.pose.position.x = self._source_pos[0]
            cur_vel_marker.pose.position.y = self._source_pos[1]
            cur_vel_marker.pose.position.z = self._source_pos[2] - 0.8

            cur_vel_marker.pose.orientation.x = qt[0]
            cur_vel_marker.pose.orientation.y = qt[1]
            cur_vel_marker.pose.orientation.z = qt[2]
            cur_vel_marker.pose.orientation.w = qt[3]

            cur_vel_marker.scale.x = 1.0
            cur_vel_marker.scale.y = 0.1
            cur_vel_marker.scale.z = 0.1

            cur_vel_marker.color.a = 1.0
            cur_vel_marker.color.r = 0.0
            cur_vel_marker.color.g = 0.0
            cur_vel_marker.color.b = 1.0
        else:
            cur_vel_marker.action = Marker.DELETE

        marker_array.markers.append(cur_vel_marker)
        return marker_array

    def update(self, t=0.0):
        """
        Plume dynamics update function. It must be implemented by the child
        class and will be used by the plume server to update the position of
        the plume particles at each iteration.
        """
        raise NotImplementedError()
