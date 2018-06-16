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
from copy import deepcopy
import numpy as np


class PlumePassiveScalarTurbulence(Plume):
    """
    Plume model implementation based on [1]. The chemical plume is described
    here by discretized particles generated that are generated on the Cartesian
    position given as the source of the plume. The plume is treated as a
    passive scalar turbulence, meaning that it will not affect the
    environmental fluid flow.

    To model the dynamics of the plume particles, the Lagrangian particle
    random walk approach [2] is used. The particles are generated from the
    source position in batches at each iteration to ensure a steady flow and
    each particle has its position :math:`(x_k, y_k, z_k)` at the instant
    :math:`t_k` computed as

    .. math::

        x_k = x_{k - 1} + (u_a + u_i) \Delta t \\
        y_k = y_{k - 1} + (v_a + v_i) \Delta t \\
        z_k = z_{k - 1} + (w_a + w_b + w_i) \Delta t

    where :math:`(u_a, v_a, w_a)` are the particle's velocities due to the
    current velocity, :math:`(u_t, v_t, w_t)` are the particle's velocities
    due to turbulent diffusion, and :math:`w_b` is the vertical buoyant
    velocity.

    [1] Yu Tian and Aiqun Zhang, "Simulation environment and guidance system
        for AUV tracing chemical plume in 3-dimensions," 2010 2nd International
        Asia Conference on Informatics in Control, Automation and Robotics
        (CAR 2010), Mar. 2010.

    [2] M. Mestres et al., "Modelling of the Ebro River plume. Validation with
        field observations," Scientia Marina, vol. 67, no. 4, pp. 379-391,
        Dec. 2003.
    """
    LABEL = 'passive_scalar_turbulence'

    def __init__(self, turbulent_diffusion_coefficients, buoyancy_flux,
                 stability_param, source_pos, n_points, start_time,
                 max_particles_per_iter=10, max_life_time=-1):
        """
        Passive scalar turbulent plume model constructor.

        Parameters
        ----------
        turbulent_diffusion_coefficients: list
            A vector with three elements containing the turbulent diffusion
            coefficients for each degree of freedom of the particles (x, y and
            z).
        buoyancy_flux: float
            Parameter used to compute the plume rise.
        stability_param: float
            Parameter used to compute the plume rise.
        source_pos: list
            Vector containing the Cartesian coordinates for the plume source
            (represented in the ENU inertial frame).
        n_points: int
            Maximum number of particles to be generated for the plume.
        start_time: float
            Plume model creation time stamp.
        """
        Plume.__init__(self, source_pos, n_points, start_time)

        assert len(turbulent_diffusion_coefficients) == 3, 'There should be ' \
            'three elements in the turbulent diffusion coefficients vector'

        assert buoyancy_flux >= 0, 'Buoyancy flux coefficient must be equal ' \
            'or greater than zero'

        self._turbulent_diffusion_coefficients = turbulent_diffusion_coefficients

        self._pnts = None
        self._vel_turbulent_diffusion = None
        self._max_particles_per_iter = max(1, max_particles_per_iter)
        self._max_life_time = max_life_time

        self.create_particles(0)

        self._buoyancy_flux = buoyancy_flux

        self._stability_param = stability_param

    @property
    def max_particles_per_iter(self):
        """
        Return the maximum number of particles to be generated per iteration
        from the source of the plume.
        """
        return self._max_particles_per_iter

    def set_max_particles_per_iter(self, n_particles):
        """
        Set the maximum number of particles to be generated per iteration from
        the source of the plume.

        Parameters
        ----------
        n_particles: int
            Number of particles.
        """
        if n_particles > 0:
            self._max_particles_per_iter = n_particles
            return True
        else:
            print 'Number of particles per iteration must be greater than zero'
            return False

    def create_particles(self, t):
        """
        Create random number of particles for one iteration up to the given
        maximum limit and remove all particles that have left the plume's
        bounding box limits.
        """
        n_particles = int(self._max_particles_per_iter * np.random.rand())

        if self._pnts is not None:

            # Remove the particles that are outside of the limits
            p_filter = self.get_contraints_filter()
            if self._max_life_time > 0:
                p_filter = np.logical_and(p_filter, t - self._time_creation < self._max_life_time)
            if np.sum(p_filter) > 0:
                # Filter out the points outside of the limit box
                self._pnts = self._pnts[np.nonzero(p_filter)[0], :]
                self._vel_turbulent_diffusion = \
                    self._vel_turbulent_diffusion[np.nonzero(p_filter)[0], :]
                self._time_creation = \
                    self._time_creation[np.nonzero(p_filter)[0]]

            if self._pnts.shape[0] + n_particles > self._n_points:
                n_particles = self._n_points - self._pnts.shape[0]

            if self._pnts.shape[0] == self._n_points:
                return

        new_pnts = np.vstack(
            (self._source_pos[0] * np.ones(n_particles),
             self._source_pos[1] * np.ones(n_particles),
             self._source_pos[2] * np.ones(n_particles))).T

        if self._pnts is None:
            self._pnts = new_pnts
            self._vel_turbulent_diffusion = np.zeros(new_pnts.shape)
            self._time_creation = self._t * np.ones(new_pnts.shape[0])
        else:
            self._pnts = np.vstack((self._pnts, new_pnts))
            self._vel_turbulent_diffusion = np.vstack(
                (self._vel_turbulent_diffusion, np.zeros(new_pnts.shape)))
            self._time_creation = np.hstack(
                (self._time_creation, self._t * np.ones(new_pnts.shape[0])))

    def set_plume_particles(self, t, x, y, z, time_creation):
        self._pnts = np.zeros(shape=(len(x), 3))
        self._time_creation = np.zeros(len(time_creation))

        self._time_creation = np.array(time_creation)
        self._time_creation -= np.max(time_creation)
        self._time_creation += t

        self._pnts[:, 0] = np.array(x)
        self._pnts[:, 1] = np.array(y)
        self._pnts[:, 2] = np.array(z)

        self._vel_turbulent_diffusion = np.zeros(self._pnts.shape)

    def compute_plume_rise(self, t):
        """
        The plume rise equation is used to compute the vertical buoyant
        velocity. It is based on the experimental results presented in [1]
        and can be written as

        .. math::

            H(u, s, t) = 2.6 \left ( \frac{F t^2}{u} \right )^{1/3} (t^2 s + 4.3)^{-1/3}

        where :math:`F` is the buoyancy flux parameter and :math:`s` the
        stability parameters, and both can be tuned by the user. :math:`u` is
        the magnitude of the current velocity on the horizontal plane. The
        resulting vertical buoyant velocity will be computed as follows

        .. math::

            w_b = \frac{H(u, s, t + \Delta t) - H(u, s, t)}{\Delta t}

        [1] Anfossi, Domenico. "Analysis of plume rise data from five TVA steam
            plants." Journal of climate and applied meteorology 24.11 (1985):
            1225-1236.

        Parameters
        ----------
        t: float
            Current simulation time stamp.
        """
        horz_vel = np.sqrt(self._current_vel[0]**2 + self._current_vel[1]**2)

        if horz_vel == 0:
            return 0.0
        t2 = np.power(t - self._time_creation, 2)
        return 2.6 * np.multiply(
            np.power(self._buoyancy_flux * t2 / horz_vel, 1. / 3),
            np.power(t2 * self._stability_param + 4.3, -1. / 3))

    def update(self, t=0.0):
        """
        Update the position of all particles and create/remove particles from
        the plume according to the bounding box limit constraints and the
        maximum number of particles allowed.

        Parameters
        ----------
        t: float
            Current simulation time stamp.
        """
        t -= self._start_time
        self._dt = t - self._t
        self._t = t
        if self._dt <= 0.0:
            print 'Time step must be greater than zero'
            return False

        self.create_particles(t)
        # Update turbulent diffusion velocity component
        for i in range(3):
            # Calculate particle velocity due to turbulent diffusion
            self._vel_turbulent_diffusion[:, i] = \
                2 * (0.5 - np.random.rand(
                    self._vel_turbulent_diffusion.shape[0])) * \
                np.sqrt(
                    6 * self._turbulent_diffusion_coefficients[i] / self._dt)
            self._pnts[:, i] += self._vel_turbulent_diffusion[:, i] * self._dt

            # Add the current velocity component
            self._pnts[:, i] += self._current_vel[i] * self._dt

            if i == 2:
                self._pnts[:, i] += \
                    (self.compute_plume_rise(self._t + self._dt) -
                     self.compute_plume_rise(self._t))
                self._pnts[np.nonzero(self._pnts[:, 2].flatten() >= 0)[0], i] = 0.0

        return True
