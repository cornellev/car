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

from .transport import DEMO_KEY, runtime


class MinimalSubscriber:
    def __init__(self, session):
        self.logger = logging.getLogger("minimal_subscriber")
        self.subscription = session.declare_subscriber(DEMO_KEY, self.listener_callback)

    def listener_callback(self, sample):
        if sample.kind != zenoh.SampleKind.PUT:
            return
        try:
            self.logger.info('I heard: "%s"', sample.payload.to_string())
        except ValueError:
            self.logger.warning("Failed to decode demo message")


def main(args=None):
    with runtime() as (session, stop):
        node = MinimalSubscriber(session)
        try:
            stop.wait()
        finally:
            node.subscription.undeclare()


if __name__ == "__main__":
    main()
