"""Client for crane_ml's sim-only reset of the boat to the course start (std_srvs/Trigger)."""


class StartReset:
    """``reset(timeout)`` -> (success, message); TimeoutError when the service is not there."""

    def __init__(self, ros, service):
        self.node, self.service, self.client = ros.node, service, None

    def __call__(self, timeout):
        from std_srvs.srv import Trigger
        from mhseals_testing.ros.nav2 import wait
        if self.client is None:
            self.client = self.node.create_client(Trigger, self.service)
        if not self.client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'{self.service} is not available')
        reply = wait(self.client.call_async(Trigger.Request()), timeout, self.service)
        return reply.success, reply.message
