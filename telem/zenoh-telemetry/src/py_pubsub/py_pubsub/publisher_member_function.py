# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import logging

import zenoh

from .transport import DEMO_KEY, run_periodic, runtime


class MinimalPublisher:
    def __init__(self, session):
        self.publisher_ = session.declare_publisher(
            DEMO_KEY,
            encoding=zenoh.Encoding.TEXT_PLAIN,
            congestion_control=zenoh.CongestionControl.BLOCK,
        )
        self.logger = logging.getLogger("minimal_publisher")
        self.i = 0

    def timer_callback(self):
        payload = f"Hello World: {self.i}"
        self.publisher_.put(payload)
        self.logger.info('Publishing: "%s"', payload)
        self.i += 1


def main(args=None):
    with runtime() as (session, stop):
        node = MinimalPublisher(session)
        try:
            run_periodic(node.timer_callback, 0.5, stop)
        finally:
            node.publisher_.undeclare()


if __name__ == "__main__":
    main()
