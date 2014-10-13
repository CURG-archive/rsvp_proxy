from gen_proto import get_option_selection_pb2
from gen_proto import get_option_selection_rpcz
from gen_proto import image_pb2
from gen_proto import compressed_image_pb2

from base_service import BaseService
from rsvp_msgs.msg import RankAction, RankGoal
from sensor_msgs.msg import Image as ImageMsg, CompressedImage as CompressedImageMsg

import rospy
import rpcz
import actionlib
import threading

class OptionSelectionService(get_option_selection_rpcz.GetOptionSelectionService, BaseService):
    def __init__(self):
        super(OptionSelectionService, self).__init__()
        rospy.loginfo('GetOptionSelectionService initialized')
        self.client = actionlib.SimpleActionClient('rank', RankAction)
        self.client.wait_for_server()

    def build_response(self, request):
        rospy.loginfo('GetOptionSelectionService build response start')

        goal = RankGoal()

        for img_opt in request.imageOptions:
            msg = ImageMsg(height=img_opt.option.height,
                           width=img_opt.option.width,
                           encoding=image_pb2.PixelFormat.Name(img_opt.option.pixel_format),
                           step=img_opt.option.step,
                           data=img_opt.option.data)

            goal.option_ids.append(img_opt.description.id)

            goal.imgs.append(msg)
            goal.descriptions.append(img_opt.description.description)
            goal.costs.append(img_opt.description.cost)

        for c_img_opt in request.compressedImageOptions:
            msg = CompressedImageMsg(format=c_img_opt.option.format,
                                     data=c_img_opt.option.data)

            goal.option_ids.append(c_img_opt.description.id)
            goal.compressed_imgs.append(msg)

            goal.descriptions.append(c_img_opt.description.description)
            goal.costs.append(c_img_opt.description.cost)

        for s_opt in request.stringOptions:
            goal.option_ids.append(s_opt.description.id)
            goal.strs.append(s_opt.option)

            goal.descriptions.append(s_opt.description.description)
            goal.costs.append(s_opt.description.cost)

        goal.minimum_confidence_level = request.minimumConfidenceLevel

        self.client.send_goal(goal)

        response = get_option_selection_pb2.GetOptionSelectionResponse()
        self.client.wait_for_result(rospy.Duration.from_sec(60.0))

        result = self.client.get_result()

        response.selectedOption = result.option_ids[0]
        response.confidence = result.confidences[0]
        response.interest_level = result.confidences
        response.option_ids = result.option_ids

        return response

class ServerThread(threading.Thread):
    def __init__(self, server_address, services):
        threading.Thread.__init__(self)
        self.daemon = True
        self.server_address = server_address
        self.services = services

    def run(self):
        app = rpcz.Application()
        server = rpcz.Server(app)

        for service in self.services:
            rospy.loginfo('registering service: {} as rpcz service: {}'.format(service.__class__.__name__, service.DESCRIPTOR.name))

            server.register_service(service, service.DESCRIPTOR.name)

        server.bind(self.server_address)
        rospy.loginfo('Serving requests on port {}'.format(self.server_address))
        app.run()


if __name__ == '__main__':
    rospy.init_node('get_option_selection_proxy')
    server_address = 'tcp://*:5561'
    services = (
        OptionSelectionService()
    )

    rospy.loginfo('launching get option selection proxy server')
    rpcz_server = ServerThread(server_address, services)
    rpcz_server.start()

    loop = rospy.Rate(10)
    while not rospy.is_shutdown() and rpcz_server.is_alive():
        try:
            loop.sleep()
        except (KeyboardInterrupt, SystemExit):
            break
    rpcz_server.join(1)
