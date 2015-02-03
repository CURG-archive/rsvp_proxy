from rsvp_msgs.msg import RankAction, RankResult
import actionlib
import rospy
import pygame
import sys
from image_converter import ImageConverter
import math
pygame.init()

class KeyboardOptionSelection(object):
    def __init__(self):
        self.size = (1024, 768)
        self.window = pygame.display.set_mode(self.size)
        pygame.display.set_caption('Keyboard Get Option Selection')
        self.screen = pygame.display.get_surface()

        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()
        self.running = True
        self.last_sent_option = 0
        self.options = None

        self.action_server = actionlib.SimpleActionServer('rank', RankAction, self.keyboard_rank_cb, False)
        self.action_server.start()

    def keyboard_rank_cb(self, msg):
        rospy.loginfo("received msg")
        self.screen.fill((0,0,0))

        scaled_imgs = [ImageConverter.from_ros(img) for img in msg.compressed_imgs]

        self.options = zip(msg.option_ids, scaled_imgs)

        elements_per_side = int(math.ceil(math.sqrt(len(self.options))))
        grid_w = self.size[0] / elements_per_side
        grid_h = self.size[1] / elements_per_side

        counter = 0

        for idx in range(len(self.options)):
            row = counter / elements_per_side
            col = counter % elements_per_side
            rect = pygame.Rect((row * grid_w, col * grid_h), (grid_w, grid_h))
            subsurf = self.screen.subsurface(rect)
            subsurf.blit(pygame.transform.smoothscale(self.options[idx][1], (grid_w, grid_h)), (0, 0))
            counter += 1

        pygame.display.flip()

        while self.options and self.running:
            pygame.time.wait(100)

    def clear(self):
        self.screen.fill((0,0,0))
        self.screen.blit(self.font.render('Sent option {}'.format(self.last_sent_option), 1, (255,255,255)), (40, self.size[1] / 2))
        pygame.display.flip()

    def main(self):
        rospy.loginfo("started main loop")
        while self.running and not rospy.is_shutdown():
            self.clock.tick(60)

            for evt in pygame.event.get():
                if evt.type == pygame.QUIT:
                    self.running = False
                    return
                elif evt.type == pygame.KEYDOWN:

                    numbers = [pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9]

                    if evt.key in (pygame.K_ESCAPE, pygame.K_q):
                        self.running = False
                        return
                    elif evt.key in numbers:
                        if not self.options:
                            continue
                        option = numbers.index(evt.key)
                        rr = RankResult()
                        rr.confidences = [0] * len(self.options)

                        options = [self.options[option]] + self.options[:option] + self.options[option+1:]
                        rr.option_ids = [x[0] for x in options]
                        self.options = None
                        self.action_server.set_succeeded(rr)
                        self.last_sent_option = option
                        self.clear()

if __name__ == '__main__':
    rospy.init_node('keyboard_get_option_selection')

    KeyboardOptionSelection().main()
    rospy.loginfo('quitting')
    pygame.quit()
    rospy.signal_shutdown('quitting')
    sys.exit()
