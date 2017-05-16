import trollius
from trollius import From

import pygazebo

class GazeboTopicPublisher(object):
    def __init__(self,topic_name, message_type):
    	# The message type used for this gazebo topic
    	self.default_message_contents = None
    	self.connected = False
    	self.topic_name = topic_name
    	self.message_type = message_type

    @trollius.coroutine
    def publishMessageLoop(self, message):
    	if not self.connected:
            self.manager = yield From (pygazebo.connect())
            self.publisher = yield From (self.manager.advertise(self.topic_name, self.message_type))
            yield From (trollius.sleep(0.5))
        
        pub_res = self.publisher.publish(message)
        res = yield From (pub_res)
        self.connected = True

    def publishMessage(self,message):
        publisher_message_loop = trollius.get_event_loop()
        publisher_message_loop.run_until_complete(self.publishMessageLoop(message))