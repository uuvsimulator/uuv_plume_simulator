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
import numpy as np


class GaussMarkovProcess(object):
    def __init__(self):
        self._mean = 0
        self._min = -1
        self._max = 1
        self._mu = 0
        self._var = 0
        self._noise_amp = 0
        self._last_time_stamp = -1

    def __str__(self):
        msg = 'Mean=%.3f\n' % self._mean
        msg += 'Min. limit=%.3f\n' % self._min
        msg += 'Max. limit=%.3f\n' % self._max
        msg += 'Mu=%.3f\n' % self._mu
        msg += 'Noise amplitude=%.3f\n' % self._noise_amp
        return msg

    @property
    def mean(self):
        return self._mean

    @property
    def min_value(self):
        return self._min

    @property
    def max_value(self):
        return self._max

    @property
    def mu(self):
        return self._mu

    @property
    def noise_amp(self):
        return self._noise_amp

    def set_model(self, mean, min_value, max_value, mu, noise_amp):
        assert min_value < max_value, 'Min. value must be smaller than max. value'
        assert min_value < mean < max_value, 'Mean value must be in the interval delimited by min. and max. value'
        assert mu >= 0, 'Mu must be greater or equal to zero'
        assert noise_amp >= 0 'Noise amplitude must be greater or equal to zero'

        self._mean = mean
        self._min = min_value
        self._max = max_value
        self._mu = mu
        self._noise_amp = noise_amp
        self._var = mean

    def set_limits(self, min_value, max_value, mean=None):
        assert min_value < max_value
        if mean is not None:
            assert min_value < mean < max_value
        else:
            assert min_value < self._mean < max_value

        self._min = min_value
        self._max = max_value

        if mean is not None:
            self._mean = mean
            self._var = mean

    def set_mu(self, mu):
        assert mu >= 0
        self._mu = mu

    def set_noise_amp(self, noise_amp):
        assert noise_amp >= 0
        self._noise_amp = noise_amp

    def update(self, t):
        if self._last_time_stamp == -1:
            self._last_time_stamp = t

        step = t - self._last_time_stamp
        if step <= 0:
            return self._var

        random = np.random.rand() - 0.5
        self._var = (1 - step * self._mu) * self._var + self._noise_amp * random

        self._var = min(self._var, self._max)
        self._var = max(self._var, self._min)

        self._last_time_stamp = t
        return self._var
