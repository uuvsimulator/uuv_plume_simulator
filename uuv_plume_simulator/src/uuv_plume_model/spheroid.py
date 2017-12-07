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

from plume import Plume
import numpy as np
from tf.transformations import quaternion_matrix


class PlumeSpheroid(Plume):
    """
    Plume model to generate a static plume with spheroid form.
    """
    LABEL = 'spheroid'

    def __init__(self, a, c, orientation, source_pos, n_points, start_time):
        """
        """
        Plume.__init__(self, source_pos, n_points, start_time)

        assert a > 0 and c > 0, 'The length of the principal axes must be ' \
            'greater than zero'
        assert orientation.size == 4, 'Orientation vector must be a quaternion'
        # Set the length of the principal axes
        self._a = a
        self._c = c

        # Store the orientation
        self._rot = orientation

    def update(self, t=0.0):
        if self._pnts is not None:
            return True

        self._t = t
        theta_range = np.linspace(-np.pi / 2, np.pi / 2, self._n_points)
        phi_range = np.linspace(-np.pi, np.pi, self._n_points)

        theta, phi = np.meshgrid(theta_range, phi_range)
        # Generate the point cloud for the spheroid
        self._pnts = np.vstack(
            (self._a * np.multiply(np.cos(theta.flatten()),
                                   np.cos(phi.flatten())),
             self._a * np.multiply(np.cos(theta.flatten()),
                                   np.sin(phi.flatten())),
             self._c * np.sin(theta.flatten()))).T

        self._time_creation = t * np.ones(self._pnts.shape[0])

        self._pnts = None
        # Change the point cloud's orientation
        # Calculate the rotation matrix first
        rot_matrix = quaternion_matrix(self._rot)[0:3, 0:3]
        for t, p in zip(theta.flatten(), phi.flatten()):
            vec = [self._a * np.cos(t) * np.cos(p),
                   self._a * np.cos(t) * np.sin(p),
                   self._c * np.sin(t)]
            vec = np.dot(rot_matrix, vec)
            if self._pnts is None:
                self._pnts = np.array(vec)
            else:
                self._pnts = np.vstack((self._pnts, vec))

        # Translate the point cloud to the correct position
        for i in range(len(self._source_pos)):
            self._pnts[:, i] += self._source_pos[i]

        self.apply_constraints()
        return True
